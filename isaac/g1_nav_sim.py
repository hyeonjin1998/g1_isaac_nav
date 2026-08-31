"""Phase 2: G1 + ROS 2 인터페이스 (프로세스 A 메인).

Phase 1 에서 검증한 정책 러너에 ROS 2 입출력을 붙입니다.

    구독:  /cmd_vel   (geometry_msgs/Twist)
    발행:  /clock, /odom, /tf (odom→base_link, base_link 하위 센서 프레임)

센서 프레임은 G1 공식 URDF 의 실장 위치를 그대로 씁니다 (``isaac/sensors.py``).

사용법::

    # 터미널 A
    source <repo>/scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --headless

    # 터미널 B
    source <repo>/scripts/ros_env.sh
    ros2 topic list
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
    ros2 run tf2_tools view_frames

``--selftest`` 를 주면 ROS 없이 내부에서 명령을 주입해 파이프라인만 검증합니다.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

# G1 USD 는 Unitree 배포 에셋이라 이 저장소에 포함돼 있지 않습니다.
# 다른 머신에서는 G1_USD 환경변수나 --usd 로 지정하세요 (README '4. G1 로봇 모델 받기').
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
parser.add_argument("--headless", action="store_true")
parser.add_argument("--usd", default=DEFAULT_USD)
parser.add_argument(
    "--scene",
    default="",
    help="환경 USD. 'warehouse'/'full_warehouse'/'warehouse_shelves' 단축어 또는 USD 경로. "
    "비우면 무한 평면(ground plane).",
)
parser.add_argument(
    "--spawn",
    type=float,
    nargs=3,
    metavar=("X", "Y", "YAW"),
    default=[0.0, 0.0, 0.0],
    help="G1 초기 위치와 요각 [m, m, rad]",
)
parser.add_argument("--policy", default=str(DEFAULT_POLICY))
parser.add_argument("--device", default="cuda")
parser.add_argument("--duration", type=float, default=0.0, help="0 이면 무한 실행")
parser.add_argument("--no-shim", action="store_true", help="보행 개시 shim 비활성화")
parser.add_argument("--camera", action="store_true", help="D435i RGB-D 스트림 발행 (렌더링 필요)")
parser.add_argument("--lidar", action="store_true", help="MID-360 포인트클라우드 발행 (렌더링 필요)")
parser.add_argument(
    "--render-hz",
    type=float,
    default=0.0,
    help="렌더 주기 [Hz, 시뮬시간]. 0이면 자동(센서 사용 시 10Hz=MID-360 실기 사양, "
    "GUI 전용이면 30Hz). 센서 발행률이 이 값을 따릅니다.",
)
parser.add_argument("--cam-width", type=int, default=640)
parser.add_argument("--cam-height", type=int, default=480)
parser.add_argument("--selftest", action="store_true", help="ROS 없이 내부 명령으로 파이프라인 검증")
parser.add_argument("--out", default="/tmp/g1_nav_sim.json")
args = parser.parse_args()

import sys  # noqa: E402

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

# 카메라/라이다 스트림은 ROS 2 브리지 확장의 replicator writer 로 나갑니다.
# 확장을 켜지 않으면 writer 가 레지스트리에 등록되지 않아
# `No writer with name 'LdrColorSDROS2PublishImage' was found` 로 실패합니다.
if args.camera or args.lidar:
    from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

    if not enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError("isaacsim.ros2.bridge 확장을 활성화하지 못했습니다")
    simulation_app.update()
    simulation_app.update()

import json  # noqa: E402
import traceback  # noqa: E402

import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from g1_policy import G1VelocityPolicy, build_joint_index_map, quat_rotate_inverse  # noqa: E402
from sensors import setup_sensor_frames  # noqa: E402

SCENE_SHORTCUTS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "full_warehouse": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "warehouse_shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "warehouse_forklifts": "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "office": "/Isaac/Environments/Office/office.usd",
    "simple_room": "/Isaac/Environments/Simple_Room/simple_room.usd",
}

PHYSICS_DT = 1.0 / 200.0
INITIAL_HEIGHT = 0.80
FALL_HEIGHT = 0.45


def main() -> int:
    import rclpy

    from ros_io import G1RosBridge, GaitCommandShim

    policy = G1VelocityPolicy(args.policy, device=args.device)
    decimation = int(round(policy.cfg.step_dt / PHYSICS_DT))
    shim = None if args.no_shim else GaitCommandShim()

    # --- 씬 ---
    # rendering_dt 는 반드시 physics_dt 와 같아야 합니다.
    # world.step(render=True) 는 내부적으로 app.update() 를 호출하고, 이는 물리를
    # rendering_dt 만큼(= rendering_dt/physics_dt 스텝) 진행시킵니다.
    # 둘이 다르면 렌더한 스텝에서만 물리가 여러 번 진행되어 루프의 스텝 카운트와
    # 실제 물리 시간이 어긋나고, 정책 제어 주기가 그 배율만큼 느려집니다.
    # (1/50 vs 1/200 이었을 때 GUI 모드에서 정책이 12.5Hz 로 돌아 보행이 발산했습니다.)
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT, stage_units_in_meters=1.0)

    if args.scene:
        from isaacsim.storage.native import get_assets_root_path

        path = SCENE_SHORTCUTS.get(args.scene, args.scene)
        if path.startswith("/Isaac/"):
            root = get_assets_root_path()
            if root is None:
                raise RuntimeError("Isaac 에셋 서버에 접근할 수 없습니다 (네트워크 확인)")
            path = root + path
        print(f"[scene] 로딩 중: {path}")
        scene_prim = define_prim("/World/Scene", "Xform")
        scene_prim.GetReferences().AddReference(path)
        # 환경 USD 가 자체 바닥을 가지므로 ground plane 은 추가하지 않습니다.
    else:
        world.scene.add_default_ground_plane()

    sx, sy, syaw = args.spawn
    prim = define_prim("/World/G1", "Xform")
    prim.GetReferences().AddReference(args.usd)
    robot = SingleArticulation(
        prim_path="/World/G1/pelvis",
        name="g1",
        position=np.array([sx, sy, INITIAL_HEIGHT]),
        orientation=np.array([np.cos(syaw / 2), 0.0, 0.0, np.sin(syaw / 2)]),
    )
    world.scene.add(robot)

    sensor_paths = setup_sensor_frames("/World/G1", add_camera=True, verify=True)
    print(f"[sensors] URDF 값과 일치 확인: {list(sensor_paths)}")

    world.reset()
    robot.initialize()

    dof_names = list(robot.dof_names)
    p2s = build_joint_index_map(dof_names)
    kp_sim = np.zeros(len(dof_names), dtype=np.float32)
    kd_sim = np.zeros(len(dof_names), dtype=np.float32)
    qdef_sim = np.zeros(len(dof_names), dtype=np.float32)
    kp_sim[p2s] = policy.cfg.stiffness
    kd_sim[p2s] = policy.cfg.damping
    qdef_sim[p2s] = policy.cfg.default_joint_pos
    robot._articulation_view.set_gains(kps=kp_sim[None, :], kds=kd_sim[None, :])
    robot.set_joint_positions(qdef_sim)
    robot.set_joint_velocities(np.zeros(len(dof_names), dtype=np.float32))
    policy.reset()

    # 카메라 실제 월드 높이를 찍어둡니다 (URDF 이식이 맞았는지 눈으로 확인).
    from isaacsim.core.utils.prims import get_prim_at_path
    from pxr import UsdGeom

    cam_xform = UsdGeom.Xformable(get_prim_at_path(sensor_paths["camera"]))
    cam_world = cam_xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    print(f"[sensors] d435 카메라 월드 위치 = ({cam_world[0]:.3f}, {cam_world[1]:.3f}, {cam_world[2]:.3f}) m")

    # --- ROS ---
    # rclpy 는 기본적으로 SIGINT/SIGTERM 핸들러를 설치해 **컨텍스트를 shutdown** 시킵니다.
    # 그러면 다른 노드를 정리하려고 보낸 신호가 이 프로세스에 잘못 닿았을 때
    # 시뮬레이터가 통째로 죽습니다 (실측: 6분 만에
    #   RCLError: Failed to publish: publisher's context is invalid → Simulation App Shutting Down).
    # 신호 처리는 SimulationApp / KeyboardInterrupt 에 맡기고 rclpy 핸들러는 끕니다.
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    bridge = G1RosBridge()

    camera_pub = None
    if args.camera:
        from ros_camera import D435Publisher

        camera_pub = D435Publisher(
            sensor_paths["camera"], resolution=(args.cam_width, args.cam_height)
        )
        print(f"[camera] {camera_pub.describe()}")

    lidar_pub = None
    if args.lidar:
        from ros_lidar import MID360Publisher

        lidar_pub = MID360Publisher(sensor_paths["mid360_link"])
        print(f"[lidar] {lidar_pub.describe()}")

    base_prim = get_prim_at_path("/World/G1/pelvis")
    cam_link_prim = get_prim_at_path(sensor_paths["d435_link"])
    lidar_link_prim = get_prim_at_path(sensor_paths["mid360_link"])

    # 렌더 데시메이션: 물리 200Hz → 지정 Hz
    sensors_on = args.camera or args.lidar
    render_on = sensors_on or (not args.headless)
    render_hz = args.render_hz if args.render_hz > 0 else (10.0 if sensors_on else 30.0)
    render_decim = max(1, int(round((1.0 / PHYSICS_DT) / render_hz)))
    if render_on:
        print(f"[render] {render_hz:.0f} Hz (물리 {render_decim} 스텝마다)"
              + ("  ← 센서 발행률" if sensors_on else "  ← 뷰포트만"))

    ros_alive = True
    q_target_sim = qdef_sim.copy()
    step = 0
    n_total = int(args.duration / PHYSICS_DT) if args.duration > 0 else -1
    stats = {"steps": 0, "cmd_received": 0, "fell": False}
    start_xy = None
    last_xy = None

    print("[run] 시뮬레이션 시작 — /cmd_vel 대기 중")

    while simulation_app.is_running():
        if n_total > 0 and step >= n_total:
            break
        # 타임스탬프는 **반드시 Isaac 내부 시뮬레이션 시간**을 써야 합니다.
        # replicator writer(카메라/라이다)는 IsaacReadSimulationTime 으로 스탬프를 찍는데,
        # 여기서 step*dt 로 자체 계산하면 두 시계가 어긋나 RTAB-Map 이
        # "Lookup would require extrapolation into the future" 로 TF 를 못 찾고
        # 노드를 하나도 만들지 못합니다 (실측: 15초 이상 벌어짐).
        # 컨텍스트가 죽으면 publish 가 예외를 던져 시뮬이 통째로 중단됩니다.
        # 조용히 죽지 않도록 원인을 한 번만 알리고 ROS I/O 만 포기합니다.
        if ros_alive and not rclpy.ok():
            print("[오류] rclpy 컨텍스트가 무효화되었습니다 — ROS I/O 를 중단하고 시뮬만 계속합니다.")
            print("       외부에서 이 프로세스로 SIGINT/SIGTERM 이 전달됐을 가능성이 큽니다.")
            ros_alive = False

        t = float(world.current_time)
        # 물리 시간과 루프 시간이 어긋나면 정책 제어 주기가 틀어져 보행이 발산합니다.
        # 조용히 실패하는 유형이라 상시 감시합니다.
        if step == 2000:
            ratio = t / (step * PHYSICS_DT)
            eff_hz = (1.0 / policy.cfg.step_dt) / ratio
            if abs(ratio - 1.0) > 0.02:
                print(f"[경고] 물리 시간이 루프 시간의 {ratio:.2f}배로 진행 중 "
                      f"→ 정책 실효 주파수 {eff_hz:.1f}Hz (목표 50Hz). 보행이 불안정해집니다.")
            else:
                print(f"[diag] 제어 주기 정상 (ratio={ratio:.3f}, 실효 {eff_hz:.1f}Hz)")
        if ros_alive:
            bridge.publish_clock(t)

        if step % decimation == 0:
            if ros_alive:
                bridge.spin_once()

            pos_w, quat_w = robot.get_world_pose()
            quat_f = np.asarray(quat_w, dtype=np.float32)
            ang_vel_w = np.asarray(robot.get_angular_velocity(), dtype=np.float32)
            lin_vel_b = quat_rotate_inverse(quat_f, np.asarray(robot.get_linear_velocity(), dtype=np.float32))
            ang_vel_b = quat_rotate_inverse(quat_f, ang_vel_w)

            if args.selftest:
                # ROS 없이: 2초 정지 → 전진 → 선회 순으로 내부 주입
                raw = np.array([0.0, 0.0, 0.0], np.float32)
                if t > 6.0:
                    raw = np.array([0.4, 0.0, 0.4], np.float32)
                elif t > 2.0:
                    raw = np.array([0.5, 0.0, 0.0], np.float32)
            else:
                raw = bridge.get_command()
                if not bridge.command_is_stale:
                    stats["cmd_received"] += 1

            if shim is not None:
                shim.update_state(lin_vel_b, float(ang_vel_b[2]))
                cmd = shim(raw)
            else:
                cmd = raw
            policy.set_command(*cmd)

            q_sim = np.asarray(robot.get_joint_positions(), dtype=np.float32)
            qd_sim = np.asarray(robot.get_joint_velocities(), dtype=np.float32)
            q_target_sim[p2s] = policy.step(quat_f, ang_vel_w, q_sim[p2s], qd_sim[p2s])

            # base_link → d435_link 는 허리 관절 때문에 고정이 아니므로 매번 계산합니다.
            t_base = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(0)
            t_cam = UsdGeom.Xformable(cam_link_prim).ComputeLocalToWorldTransform(0)
            t_rel = t_cam * t_base.GetInverse()
            rel_t = t_rel.ExtractTranslation()
            rel_q = t_rel.ExtractRotationQuat()
            rel_im = rel_q.GetImaginary()
            bridge.set_camera_transform(
                np.array([rel_t[0], rel_t[1], rel_t[2]], dtype=np.float64),
                np.array([rel_q.GetReal(), rel_im[0], rel_im[1], rel_im[2]], dtype=np.float64),
            )

            # MID-360 도 torso_link 에 붙어 있어 base_link 기준 변환이 고정이 아닙니다.
            t_lid = UsdGeom.Xformable(lidar_link_prim).ComputeLocalToWorldTransform(0)
            l_rel = t_lid * t_base.GetInverse()
            l_t = l_rel.ExtractTranslation()
            l_q = l_rel.ExtractRotationQuat()
            l_im = l_q.GetImaginary()
            bridge.set_lidar_transform(
                np.array([l_t[0], l_t[1], l_t[2]], dtype=np.float64),
                np.array([l_q.GetReal(), l_im[0], l_im[1], l_im[2]], dtype=np.float64),
            )

            if ros_alive:
                bridge.publish_odom(np.asarray(pos_w, np.float64), quat_f, lin_vel_b, ang_vel_b)

            if start_xy is None:
                start_xy = (float(pos_w[0]), float(pos_w[1]))
            last_xy = (float(pos_w[0]), float(pos_w[1]))

            if pos_w[2] < FALL_HEIGHT:
                print(f"[run] !! 낙상: t={t:.2f}s")
                stats["fell"] = True
                break

        robot.apply_action(ArticulationAction(joint_positions=q_target_sim))
        # 카메라 writer 는 렌더 파이프라인이 돌 때만 프레임을 내보냅니다 (headless 여도 필요).
        # 센서 writer 는 렌더 프레임마다 발행하므로 렌더 주기가 곧 센서 주기입니다.
        # GUI 여도 매 스텝 렌더하지 않습니다 (rendering_dt==physics_dt 라 물리는
        # 어느 쪽이든 1스텝씩만 진행되지만, 200FPS 렌더는 불필요하게 무겁습니다).
        do_render = render_on and (step % render_decim == 0)
        world.step(render=do_render)
        step += 1

    stats["steps"] = step
    stats["sim_time"] = step * PHYSICS_DT
    if start_xy and last_xy:
        stats["start_xy"] = [round(v, 3) for v in start_xy]
        stats["end_xy"] = [round(v, 3) for v in last_xy]
        stats["displacement_m"] = round(
            float(np.hypot(last_xy[0] - start_xy[0], last_xy[1] - start_xy[1])), 3
        )
    Path(args.out).write_text(json.dumps(stats, indent=2))
    print(f"[out] {stats}")
    return 0 if not stats["fell"] else 1


exit_code = 1
try:
    exit_code = main()
except KeyboardInterrupt:
    print("\n[중단] 사용자 종료 요청")
    exit_code = 0
except Exception:  # noqa: BLE001
    traceback.print_exc()
    Path(args.out).write_text(json.dumps({"verdict": "ERROR", "traceback": traceback.format_exc()}, indent=2))
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    simulation_app.close()

raise SystemExit(exit_code)
