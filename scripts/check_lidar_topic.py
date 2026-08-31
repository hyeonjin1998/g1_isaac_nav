#!/usr/bin/env python3
"""발행된 `/mid360/points` 가 실제로 MID-360 인지 검사합니다 (프로세스 B).

`isaac/check_lidar_profile.py` 는 센서 프로파일만 봅니다. 정작 맵에 들어가는 것은
ROS 2 writer 가 내보낸 **메시지 한 건**이고, RTX 라이다는 렌더 tick 사이에 지나간
방위각만 담기 때문에 둘이 다를 수 있습니다. 그래서 토픽을 직접 뜯어봅니다.

사용법::

    # 터미널 A
    source scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --scene simple_room --lidar

    # 터미널 B
    source scripts/ros_env.sh
    python3 scripts/check_lidar_topic.py

기대값
------
==================  ==========  ==========================================
항목                기대         근거
==================  ==========  ==========================================
발행 주기           10 Hz        MID-360 실기 사양
메시지당 점 개수    ~20000       200k pts/s ÷ 10 Hz. **조각 발행이면 수백 점**
메시지당 방위각     ≥ 300°       360° − 로봇 팔/어깨 자체 가림(±50° 부근 2구간)
고도 최저           −7°          프로파일 고유값. 대체 라이다는 −15°
==================  ==========  ==========================================

고도 상단(+52°)은 로봇에 달면 자체 가림·씬 천장에 좌우되므로 여기서 보지 않습니다.
센서 자체의 −7~+52° 는 `isaac/check_lidar_profile.py`(닫힌 방)가 검사합니다.
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

EXPECT_ELEV_MIN_DEG = -7.0
EXPECT_ELEV_MAX_DEG = 52.0


class LidarTopicCheck(Node):
    def __init__(self, topic: str, count: int) -> None:
        super().__init__("check_lidar_topic")
        self.want = count
        self.msgs: list[tuple[float, np.ndarray]] = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PointCloud2, topic, self._on_cloud, qos)
        self.get_logger().info(f"{topic} 대기 중 ({count}개 메시지)")

    def _on_cloud(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.msgs.append((stamp, np.asarray(pts, dtype=np.float64).reshape(-1, 3)))
        self.frame_id = msg.header.frame_id

    @property
    def done(self) -> bool:
        return len(self.msgs) >= self.want


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/mid360/points")
    ap.add_argument("--count", type=int, default=15, help="검사할 메시지 수")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--hist", action="store_true", help="방위각/고도 분포 출력 (자체 가림 확인용)")
    args = ap.parse_args()

    rclpy.init()
    node = LidarTopicCheck(args.topic, args.count)
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    t0 = __import__("time").monotonic()
    while rclpy.ok() and not node.done and __import__("time").monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)

    msgs = node.msgs
    if not msgs:
        print(f"!! {args.topic} 에서 메시지를 받지 못했습니다")
        rclpy.shutdown()
        return 1

    spans, counts, elmin, elmax, rmin, rmax = [], [], [], [], [], []
    for _stamp, pts in msgs:
        if len(pts) == 0:
            counts.append(0)
            continue
        d = np.linalg.norm(pts, axis=1)
        keep = d > 1e-3
        pts, d = pts[keep], d[keep]
        if len(pts) == 0:
            counts.append(0)
            continue
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        el = np.degrees(np.arcsin(np.clip(pts[:, 2] / d, -1.0, 1.0)))
        occupied = np.unique(np.floor((az + 180.0) / 2.0).astype(int))  # 2° 빈
        spans.append(len(occupied) * 2.0)
        counts.append(len(pts))
        elmin.append(el.min())
        elmax.append(el.max())
        rmin.append(d.min())
        rmax.append(d.max())

    if args.hist:
        pts = np.vstack([p for _s, p in msgs if len(p)])
        d = np.linalg.norm(pts, axis=1)
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        el = np.degrees(np.arcsin(np.clip(pts[:, 2] / np.maximum(d, 1e-9), -1.0, 1.0)))
        for label, vals, rng, bins in (("방위각", az, (-180, 180), 24), ("고도각", el, (-10, 55), 13)):
            hist, edges = np.histogram(vals, bins=bins, range=rng)
            print(f"\n{label} 분포:")
            for h, e in zip(hist, edges):
                print(f"  {e:+7.0f}° {h:8d} {'#' * int(50 * h / max(hist.max(), 1))}")

    stamps = [s for s, _ in msgs]
    dt = np.diff(stamps)
    print()
    print(f"메시지 {len(msgs)}개  frame_id={node.frame_id}")
    print(f"  발행 주기        : {1.0 / dt.mean():.2f} Hz (시뮬시간)" if len(dt) else "")
    print(f"  메시지당 점 개수 : {np.mean(counts):.0f}  (min {min(counts)} / max {max(counts)})")
    print(f"  메시지당 방위각  : {np.mean(spans):.1f}°  (min {min(spans):.1f}°)")
    print(f"  고도각 범위      : {min(elmin):.2f}° ~ {max(elmax):.2f}°")
    print(f"  거리 범위        : {min(rmin):.2f} ~ {max(rmax):.2f} m")

    # 로봇에 달린 상태의 기준 (자체 가림 포함). 센서 원형은 check_lidar_profile.py 담당.
    az_ok = min(spans) >= 300.0
    el_ok = abs(min(elmin) - EXPECT_ELEV_MIN_DEG) < 1.5
    # 조각 발행(렌더 tick 당 1/20 회전)이면 여기서 걸립니다 — 실측 127점 vs 정상 수천 점.
    n_ok = np.mean(counts) >= 1000
    hz_ok = len(dt) > 0 and abs(1.0 / dt.mean() - 10.0) < 1.0
    print()
    print(f"  발행 10 Hz          : {'PASS' if hz_ok else 'FAIL'}")
    print(f"  메시지당 ≥1000 점   : {'PASS' if n_ok else 'FAIL'}  (조각 발행 감지)")
    print(f"  방위각 ≥300°        : {'PASS' if az_ok else 'FAIL'}  (팔 자체 가림 제외)")
    print(f"  고도 최저 −7°       : {'PASS' if el_ok else 'FAIL'}  (대체 라이다면 −15°)")
    ok = az_ok and el_ok and n_ok and hz_ok
    print(f"\n{'PASS' if ok else 'FAIL'} — 발행되는 클라우드가 MID-360 사양과 "
          f"{'일치합니다' if ok else '다릅니다'}")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
