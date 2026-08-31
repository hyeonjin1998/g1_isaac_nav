"""Phase 1: Isaac Sim 에서 G1 이 속도 명령으로 걷는지 검증 (ROS 없음).

ROS 를 붙이기 전에 **정책 이식이 맞았는지만** 격리 검증합니다. 여기서 실패하면
관절 순서 / 관측 레이아웃 / 게인 / MuJoCo→PhysX 갭 중 하나가 원인이고,
ROS 변수를 섞으면 원인 분리가 어려워집니다.

사용법::

    source <repo>/scripts/isaac_env.sh
    python isaac/g1_walk_test.py --headless --vx 0.5 --duration 10

    # 화면으로 보기
    python isaac/g1_walk_test.py --vx 0.5

검증 기준
--------
- 10 초 이상 넘어지지 않음 (pelvis 높이 > 0.45 m 유지)
- 정상 상태 전진 속도가 명령의 ±20% 이내
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

# G1 USD 는 Unitree 배포 에셋이라 이 저장소에 포함돼 있지 않습니다.
# 다른 머신에서는 G1_USD 환경변수나 --usd 로 지정하세요 (README '0. 사전 준비').
_G1_USD_REL = "unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd"
_G1_USD_CANDIDATES = (
    Path(__file__).resolve().parents[1] / "assets" / _G1_USD_REL,  # 저장소 내부에 둔 경우
    Path.home() / _G1_USD_REL,
    Path.home() / "Project/03_IsaacPDW" / _G1_USD_REL,             # 개발 머신 배치
)


def _resolve_g1_usd() -> str:
    env = os.environ.get("G1_USD")
    if env:
        return env
    for cand in _G1_USD_CANDIDATES:
        if cand.exists():
            return str(cand)
    return str(_G1_USD_CANDIDATES[0])  # 없으면 첫 후보로 실패시켜 경로를 보여줍니다


DEFAULT_USD = _resolve_g1_usd()
DEFAULT_POLICY = Path(__file__).parent / "policy" / "velocity_v0"

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--headless", action="store_true", help="GUI 없이 실행")
parser.add_argument("--usd", default=DEFAULT_USD, help="G1 USD 경로")
parser.add_argument("--policy", default=str(DEFAULT_POLICY), help="정책 디렉터리 (policy.onnx + deploy.yaml)")
parser.add_argument("--vx", type=float, default=0.5, help="전진 속도 명령 [m/s]")
parser.add_argument("--vy", type=float, default=0.0, help="측방 속도 명령 [m/s]")
parser.add_argument("--wz", type=float, default=0.0, help="요 각속도 명령 [rad/s]")
parser.add_argument("--duration", type=float, default=10.0, help="명령 인가 후 주행 시간 [s]")
parser.add_argument("--ramp", type=float, default=2.0, help="명령을 0→목표로 올리는 시간 [s]")
parser.add_argument("--settle", type=float, default=1.0, help="명령 전 기본자세 유지 시간 [s]")
parser.add_argument("--device", default="cuda", help="정책 추론 디바이스")
parser.add_argument("--out", default="/tmp/g1_walk_test.json", help="결과 JSON 저장 경로")
args = parser.parse_args()

# SimulationApp.close() 가 os._exit() 를 호출해 stdout 버퍼와 종료 코드를 모두 날립니다.
# → 라인 버퍼링 + 결과 JSON 파일로 이중 기록. **exit code 를 신뢰하지 말 것.**
import sys as _sys  # noqa: E402

_sys.stdout.reconfigure(line_buffering=True)
_sys.stderr.reconfigure(line_buffering=True)

# SimulationApp 은 다른 isaacsim 임포트보다 반드시 먼저 생성해야 합니다.
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import json  # noqa: E402
import traceback  # noqa: E402

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

import sys  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from g1_policy import (  # noqa: E402
    G1_29DOF_JOINT_ORDER,
    G1VelocityPolicy,
    build_joint_index_map,
    quat_rotate_inverse,
)

PHYSICS_DT = 1.0 / 200.0
INITIAL_HEIGHT = 0.80
FALL_HEIGHT = 0.45


def main() -> int:
    policy = G1VelocityPolicy(args.policy, device=args.device)
    decimation = int(round(policy.cfg.step_dt / PHYSICS_DT))
    print(f"[setup] step_dt={policy.cfg.step_dt}s, physics_dt={PHYSICS_DT}s → decimation={decimation}")

    # rendering_dt 는 physics_dt 와 같아야 합니다. 다르면 render=True 인 스텝에서
    # 물리가 rendering_dt/physics_dt 배로 진행되어 정책 제어 주기가 그만큼 느려지고
    # (GUI 모드에서 4배 → 12.5Hz) 보행이 발산합니다.
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    prim = define_prim("/World/G1", "Xform")
    prim.GetReferences().AddReference(args.usd)
    robot = SingleArticulation(
        prim_path="/World/G1/pelvis",
        name="g1",
        position=np.array([0.0, 0.0, INITIAL_HEIGHT]),
    )
    world.scene.add(robot)

    world.reset()
    robot.initialize()

    # --- 관절 순서 매핑 (조용히 틀리기 가장 쉬운 지점이라 반드시 출력해서 확인) ---
    dof_names = list(robot.dof_names)
    print(f"[joints] 시뮬 DOF 수 = {len(dof_names)}")
    p2s = build_joint_index_map(dof_names)  # 정책순서[i] = 시뮬순서[p2s[i]]
    s2p = np.argsort(p2s)  # 시뮬순서[j] = 정책순서[s2p[j]]
    if list(np.arange(len(p2s))) == list(p2s):
        print("[joints] 시뮬 DOF 순서가 정책 순서와 동일합니다 (재정렬 불필요)")
    else:
        print("[joints] 순서가 다릅니다 — 이름 기반 재정렬 적용")
        for i, name in enumerate(G1_29DOF_JOINT_ORDER):
            if p2s[i] != i:
                print(f"          정책[{i:2d}] {name:32s} ← 시뮬[{p2s[i]:2d}]")

    # --- 게인 / 기본 자세를 시뮬 DOF 순서로 배치 ---
    kp_sim = np.zeros(len(dof_names), dtype=np.float32)
    kd_sim = np.zeros(len(dof_names), dtype=np.float32)
    qdef_sim = np.zeros(len(dof_names), dtype=np.float32)
    kp_sim[p2s] = policy.cfg.stiffness
    kd_sim[p2s] = policy.cfg.damping
    qdef_sim[p2s] = policy.cfg.default_joint_pos

    robot._articulation_view.set_gains(kps=kp_sim[None, :], kds=kd_sim[None, :])
    robot.set_joint_positions(qdef_sim)
    robot.set_joint_velocities(np.zeros(len(dof_names), dtype=np.float32))
    print(f"[setup] PD 게인 적용: kp {kp_sim.min():.1f}~{kp_sim.max():.1f}, kd {kd_sim.min():.1f}~{kd_sim.max():.1f}")

    policy.reset()

    # --- 시뮬레이션 루프 ---
    n_settle = int(args.settle / PHYSICS_DT)
    n_total = int((args.settle + args.ramp + args.duration) / PHYSICS_DT)
    target_cmd = np.array([args.vx, args.vy, args.wz], dtype=np.float32)
    print(f"[run] 명령 목표 = {target_cmd} (클램프 후 {policy.set_command(*target_cmd)})")

    q_target_sim = qdef_sim.copy()
    samples: list[tuple[float, np.ndarray, float, float]] = []  # (t, lin_vel_b, wz_b, height)
    fell_at: float | None = None

    for step in range(n_total):
        t = step * PHYSICS_DT

        if step % decimation == 0:
            # 명령 램프: 급격한 명령 인가는 정책이 못 따라가 넘어집니다.
            if t < args.settle:
                scale = 0.0
            elif t < args.settle + args.ramp:
                scale = (t - args.settle) / args.ramp if args.ramp > 0 else 1.0
            else:
                scale = 1.0
            policy.set_command(*(target_cmd * scale))

            pos_w, quat_w = robot.get_world_pose()
            ang_vel_w = robot.get_angular_velocity()
            q_sim = robot.get_joint_positions()
            qd_sim = robot.get_joint_velocities()

            q_target_pol = policy.step(
                np.asarray(quat_w, dtype=np.float32),
                np.asarray(ang_vel_w, dtype=np.float32),
                np.asarray(q_sim, dtype=np.float32)[p2s],
                np.asarray(qd_sim, dtype=np.float32)[p2s],
            )
            q_target_sim[p2s] = q_target_pol

            if t >= args.settle + args.ramp:
                # 월드 프레임 속도는 로봇이 회전하면 의미가 없습니다.
                # 명령이 바디 프레임 기준이므로 측정도 바디 프레임으로 변환합니다.
                quat_f = np.asarray(quat_w, dtype=np.float32)
                lin_vel_b = quat_rotate_inverse(quat_f, np.asarray(robot.get_linear_velocity(), dtype=np.float32))
                ang_vel_b = quat_rotate_inverse(quat_f, np.asarray(ang_vel_w, dtype=np.float32))
                samples.append((t, lin_vel_b, float(ang_vel_b[2]), float(pos_w[2])))

            if pos_w[2] < FALL_HEIGHT and fell_at is None:
                fell_at = t
                print(f"[run] !! 낙상 감지: t={t:.2f}s, pelvis z={pos_w[2]:.3f}m")
                break

        robot.apply_action(ArticulationAction(joint_positions=q_target_sim))
        world.step(render=not args.headless)

    # --- 결과 ---
    print("\n" + "=" * 64)
    result: dict = {"command": [args.vx, args.vy, args.wz], "fell_at": fell_at}

    if fell_at is not None:
        result["verdict"] = "FAIL_FELL"
        print(f"결과: 실패 — t={fell_at:.2f}s 에 낙상")
        print("점검 순서: 1) 관절 순서 매핑  2) obs 레이아웃  3) PD 게인  4) MuJoCo→PhysX 갭")
    elif not samples:
        result["verdict"] = "INCONCLUSIVE"
        print("결과: 판정 불가 — 정상 상태 샘플이 없습니다 (--duration 을 늘리세요)")
    else:
        vel_b = np.stack([s[1] for s in samples])
        wz_b = np.array([s[2] for s in samples])
        heights = np.array([s[3] for s in samples])
        # 정상 상태만 보려면 마지막 절반을 씁니다 (명령 인가 직후의 과도 응답 배제).
        half = len(samples) // 2
        meas = {
            "vx": float(vel_b[half:, 0].mean()),
            "vy": float(vel_b[half:, 1].mean()),
            "wz": float(wz_b[half:].mean()),
        }
        result.update(
            height_mean=float(heights.mean()),
            height_min=float(heights.min()),
            measured_body=meas,
        )
        print(f"결과: {args.settle + args.ramp + args.duration:.1f}s 완주, 낙상 없음")
        print(f"  pelvis 높이  평균 {heights.mean():.3f} m  (min {heights.min():.3f})")
        print("  바디 프레임 (정상 상태, 마지막 절반 평균):")

        # 명령이 인가된 축만 추종 판정합니다.
        axes = [("vx", args.vx, "m/s"), ("vy", args.vy, "m/s"), ("wz", args.wz, "rad/s")]
        errors: dict[str, float] = {}
        for key, cmd, unit in axes:
            got = meas[key]
            if abs(cmd) > 1e-3:
                err = abs(got - cmd) / abs(cmd)
                errors[key] = float(err)
                mark = "OK" if err <= 0.20 else "미달"
                print(f"    {key} {got:+.3f} {unit}  (명령 {cmd:+.2f}, 오차 {err * 100:5.1f}%) {mark}")
            else:
                print(f"    {key} {got:+.3f} {unit}  (명령 없음)")

        result["rel_errors"] = errors
        if not errors:
            result["verdict"] = "PASS_NO_CRITERION"
        elif max(errors.values()) <= 0.20:
            result["verdict"] = "PASS"
        else:
            result["verdict"] = "FAIL_TRACKING"
        print(f"  → {result['verdict']}")

    Path(args.out).write_text(json.dumps(result, indent=2))
    print(f"[out] 결과 저장: {args.out}")
    return 0 if result["verdict"].startswith("PASS") else 1


exit_code = 1
try:
    exit_code = main()
except Exception:  # noqa: BLE001
    # close() 가 os._exit() 를 호출하므로 여기서 직접 출력하지 않으면 트레이스백이 사라집니다.
    traceback.print_exc()
    Path(args.out).write_text(json.dumps({"verdict": "ERROR", "traceback": traceback.format_exc()}, indent=2))
finally:
    _sys.stdout.flush()
    _sys.stderr.flush()
    simulation_app.close()

raise SystemExit(exit_code)
