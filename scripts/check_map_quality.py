#!/usr/bin/env python3
"""RTAB-Map DB 품질 검사 — 맵이 뒤틀렸는지 숫자로 판정합니다.

시뮬에서는 `/odom` 이 PhysX 참값(`robot.get_world_pose()`)이므로
**루프 클로저의 오차 = 곧 오검출**이고, 최적화된 포즈가 odom 에서 멀어진 만큼이
그대로 맵 왜곡입니다. 이 두 가지를 재서 판정합니다.

사용법::

    source scripts/ros_env.sh
    python3 scripts/check_map_quality.py ros2_ws/src/g1_localization/maps/explored.db

    # 맵 왜곡까지 보려면 (rtabmap-export 필요, 수 분 걸림)
    python3 scripts/check_map_quality.py --poses <db>

판정 기준
--------
- 장소 오검출(병진 오차 > 0.3 m) 이 **0** 이어야 합니다. 1 개라도 있으면
  그 시점부터 맵 전체가 뒤틀리고, RTAB-Map 이 이후 정상 클로저까지 거부합니다.
- 장애물 셀 보유 노드 비율이 낮으면(< 50%) 라이다가 조각 발행 중일 수 있습니다
  (`scripts/check_lidar_topic.py` 로 확인).
"""

from __future__ import annotations

import argparse
import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile

import numpy as np

BAD_TRANS_M = 0.3
BAD_ROT_DEG = 15.0


def m34(blob):
    if blob is None:
        return None
    a = np.frombuffer(bytes(blob), dtype=np.float32)[:12].reshape(3, 4).astype(np.float64)
    T = np.eye(4)
    T[:3, :4] = a
    return T if abs(np.linalg.det(T[:3, :3]) - 1.0) < 1e-3 else None


def rot_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R[:3, :3]) - 1) / 2, -1, 1)))


def check_cells(cur):
    tot = cur.execute("select count(*) from Data").fetchone()[0]
    print(f"노드 {tot}")
    out = {}
    for col in ("ground_cells", "obstacle_cells", "empty_cells"):
        n = cur.execute(
            f"select count(*) from Data where {col} is not null and length({col})>0"
        ).fetchone()[0]
        out[col] = n / tot if tot else 0.0
        print(f"  {col:15s} {n:5d}/{tot} ({100 * out[col]:5.1f}%)")
    return out


def check_links(cur):
    poses = {i: m34(p) for i, p in cur.execute("select id,pose from Node").fetchall()}
    seen, rows = set(), []
    for f, t, ty, tr in cur.execute("select from_id,to_id,type,transform from Link where type in (1,2)"):
        k = (min(f, t), max(f, t))
        if k in seen:
            continue
        seen.add(k)
        rows.append((f, t, ty, tr))

    checked, bad, degenerate = 0, 0, 0
    print(f"\n루프 클로저 {len(rows)}개 (참값 odom 과 대조)")
    for f, t, ty, tr in sorted(rows):
        T = m34(tr)
        if T is None:
            degenerate += 1
            continue
        if poses.get(f) is None or poses.get(t) is None:
            continue
        err = np.linalg.inv(T) @ (np.linalg.inv(poses[f]) @ poses[t])
        dt, dr = float(np.linalg.norm(err[:3, 3])), float(rot_deg(err))
        checked += 1
        flag = dt > BAD_TRANS_M or dr > BAD_ROT_DEG
        bad += flag
        if flag or dr > 5.0:
            mark = "  <-- 장소 오검출" if flag else "  (요각 큼)"
            print(f"  {f:5d}->{t:5d} type={ty}  dt={dt:7.3f}m  dR={dr:5.2f}°{mark}")
    if degenerate:
        print(f"  (정지 구간 병합 링크 {degenerate}개 제외)")
    print(f"  => 검사 {checked}개 중 장소 오검출 {bad}개")
    return checked, bad


def check_poses(db):
    """최적화 포즈 vs odom 참값. rtabmap-export 를 두 번 돌립니다 (느림)."""
    tmp = tempfile.mkdtemp(prefix="mapq_")
    try:
        for opt, name in ((0, "opt"), (3, "odo")):
            r = subprocess.run(
                ["rtabmap-export", "--poses", "--opt", str(opt), "--output", os.path.join(tmp, name), db],
                capture_output=True, text=True,
            )
            if r.returncode != 0:
                print(f"  rtabmap-export 실패: {r.stderr.strip()[:200]}")
                return

        def load(p):
            d = {}
            for line in open(p):
                if line.startswith("#"):
                    continue
                v = line.split()
                x, y = float(v[1]), float(v[2])
                qz, qw = float(v[6]), float(v[7])
                d[int(v[8])] = (x, y, 2 * np.arctan2(qz, qw))
            return d

        opt = load(os.path.join(tmp, "opt_poses.txt"))
        odo = load(os.path.join(tmp, "odo_poses.txt"))
        ids = sorted(set(opt) & set(odo))
        if not ids:
            print("  공통 노드 없음")
            return
        i0 = ids[0]
        d0 = opt[i0][2] - odo[i0][2]
        c, s = np.cos(d0), np.sin(d0)
        errs = []
        for i in ids:
            dx, dy = odo[i][0] - odo[i0][0], odo[i][1] - odo[i0][1]
            ax, ay = c * dx - s * dy + opt[i0][0], s * dx + c * dy + opt[i0][1]
            errs.append((i, float(np.hypot(opt[i][0] - ax, opt[i][1] - ay))))

        print(f"\n맵 왜곡 (최적화 포즈 vs 참값), 공통 노드 {len(ids)}개")
        step = max(1, len(errs) // 8)
        for k in range(0, len(errs), step):
            ch = errs[k : k + step]
            e = [x[1] for x in ch]
            print(f"  노드 {ch[0][0]:5d}-{ch[-1][0]:5d}  평균 {np.mean(e):7.3f} m  최대 {max(e):7.3f} m")
        worst = max(errs, key=lambda x: x[1])
        print(f"  => 최대 왜곡 {worst[1]:.3f} m (노드 {worst[0]})")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("db")
    ap.add_argument("--poses", action="store_true", help="맵 왜곡까지 측정 (rtabmap-export, 수 분)")
    args = ap.parse_args()

    if not os.path.isfile(args.db):
        print(f"!! DB 가 없습니다: {args.db}")
        return 2

    c = sqlite3.connect(args.db)
    cur = c.cursor()
    print(f"=== {os.path.basename(args.db)} ===")
    cells = check_cells(cur)
    checked, bad = check_links(cur)
    if args.poses:
        check_poses(args.db)

    print()
    ok_links = bad == 0
    ok_cells = cells.get("obstacle_cells", 0) >= 0.5
    print(f"  장소 오검출 0      : {'PASS' if ok_links else f'FAIL ({bad}개)'}")
    print(f"  장애물 셀 ≥50%     : {'PASS' if ok_cells else 'FAIL — 라이다 조각 발행 의심'}")
    if checked == 0:
        print("  (주: 재방문이 없어 검사할 클로저가 없었습니다 — 크기만으론 판정 불가)")
    return 0 if (ok_links and ok_cells) else 1


if __name__ == "__main__":
    raise SystemExit(main())
