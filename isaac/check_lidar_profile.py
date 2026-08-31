"""MID-360 프로파일이 실제로 적용됐는지 확인하는 격리 검증 (Phase 3 회귀 방지).

배경
----
Isaac Sim 5.x 의 `IsaacSensorCreateRtxLidar` 는 커스텀 JSON 프로파일을 못 찾으면
**경고 한 줄만 남기고 기본 라이다로 대체**합니다. 로그는 잘 흘러가고 포인트도
계속 나오므로 매핑을 한참 돌린 뒤 맵이 이상한 것을 보고서야 알게 됩니다.
그래서 스캔의 실제 커버리지를 숫자로 찍어 확인합니다.

사용법::

    source <repo>/scripts/isaac_env.sh
    python isaac/check_lidar_profile.py              # 현재 구현 (legacy 카메라 prim)
    python isaac/check_lidar_profile.py --mode modern  # 대조군: 5.x 기본 경로

기대값 (Livox_MID360.json)::

    방위각 360°,  고도 −7° ~ +52°,  10 Hz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument(
    "--mode",
    choices=("legacy", "modern"),
    default="legacy",
    help="legacy=force_camera_prim(JSON 프로파일 적용), modern=5.x 기본 OmniLidar 경로",
)
parser.add_argument("--frames", type=int, default=30, help="집계할 렌더 프레임 수")
parser.add_argument("--height", type=float, default=1.24, help="라이다 장착 높이 [m] (G1 실장값)")
parser.add_argument(
    "--render-hz",
    type=float,
    default=10.0,
    help="렌더 주기 [Hz]. **g1_nav_sim.py 와 같은 값이어야 의미가 있습니다** — "
    "RTX 라이다는 렌더 tick 사이에 지나간 방위각만 쌓기 때문입니다.",
)
parser.add_argument("--hist", action="store_true", help="방위각 10° 히스토그램 출력")
parser.add_argument(
    "--step-mode",
    choices=("demo", "simple"),
    default="demo",
    help="demo=g1_nav_sim.py 와 동일(rendering_dt=physics_dt, N스텝마다 렌더), "
    "simple=rendering_dt 를 렌더 주기로 두고 매 스텝 렌더",
)
args = parser.parse_args()

sys.stdout.reconfigure(line_buffering=True)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import VisualCuboid  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from ros_lidar import CONFIG_NAME, register_config_folder  # noqa: E402

# 기대 프로파일 (Livox_MID360.json 과 일치해야 합니다)
EXPECT_AZIMUTH_DEG = 360.0
EXPECT_ELEV_MIN_DEG = -7.0
EXPECT_ELEV_MAX_DEG = 52.0

ROOM_HALF = 6.0
WALL_H = 3.0
CEIL_Z = 5.0


def build_room(world: World) -> None:
    """사방 벽 + 천장. 360°·고도 +52° 까지 반사가 잡히도록 만든 방입니다."""
    world.scene.add_default_ground_plane()
    walls = [
        (ROOM_HALF, 0.0, 0.2, 2 * ROOM_HALF),
        (-ROOM_HALF, 0.0, 0.2, 2 * ROOM_HALF),
        (0.0, ROOM_HALF, 2 * ROOM_HALF, 0.2),
        (0.0, -ROOM_HALF, 2 * ROOM_HALF, 0.2),
    ]
    for i, (x, y, sx, sy) in enumerate(walls):
        VisualCuboid(
            prim_path=f"/World/wall_{i}",
            position=np.array([x, y, WALL_H / 2]),
            scale=np.array([sx, sy, WALL_H]),
        )
    VisualCuboid(
        prim_path="/World/ceiling",
        position=np.array([0.0, 0.0, CEIL_Z]),
        scale=np.array([2 * ROOM_HALF, 2 * ROOM_HALF, 0.2]),
    )


def make_lidar(parent_path: str, mode: str):
    """`ros_lidar.MID360Publisher` 와 동일한 인자로 라이다를 만듭니다."""
    register_config_folder()
    kwargs = dict(path="mid360", parent=parent_path, config=CONFIG_NAME)
    if mode == "legacy":
        kwargs["force_camera_prim"] = True
    result, prim = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", **kwargs)
    if not result or prim is None or not prim.IsValid():
        raise RuntimeError("RTX 라이다 생성 실패")
    return prim


def main() -> int:
    # RTX 라이다의 스캔 진행량은 렌더 tick 당 얼마의 시간이 흐르는가에 달려 있어
    # **스텝 방식이 곧 스캔 커버리지를 결정합니다.** 그래서 데모와 같은 방식이 기본값입니다.
    physics_dt = 1.0 / 200.0
    render_decim = max(1, int(round((1.0 / physics_dt) / args.render_hz)))
    rendering_dt = physics_dt if args.step_mode == "demo" else 1.0 / args.render_hz
    world = World(physics_dt=physics_dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
    build_room(world)

    # mid360_link 를 흉내낸 부모 (G1 실장 높이, 자세는 항등)
    link = define_prim("/World/mid360_link", "Xform")
    UsdGeom.Xformable(link).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, args.height))

    prim = make_lidar("/World/mid360_link", args.mode)
    cfg_attr = prim.GetAttribute("sensorModelConfig")
    applied = cfg_attr.Get() if cfg_attr and cfg_attr.IsValid() else None
    print(f"[prim] path={prim.GetPath()} type={prim.GetTypeName()} sensorModelConfig={applied!r}")

    render_product = rep.create.render_product(
        prim.GetPath().pathString,
        [1, 1],
        name="LidarCheckRP",
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
    )
    annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
    annot.initialize(outputAzimuth=True, outputElevation=True, outputDistance=True)
    annot.attach([render_product.path])

    world.reset()

    frames = []  # 프레임별 (점 개수, 방위각 span, 고도 min, 고도 max)
    allpts = []
    for _ in range(args.frames):
        if args.step_mode == "demo":
            for _ in range(render_decim - 1):
                world.step(render=False)
        world.step(render=True)
        data = annot.get_data()
        pts = data.get("data") if data else None
        if pts is None or len(pts) == 0:
            continue
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        d = np.linalg.norm(pts, axis=1)
        keep = d > 1e-3
        pts, d = pts[keep], d[keep]
        if len(pts) == 0:
            continue
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        el = np.degrees(np.arcsin(np.clip(pts[:, 2] / d, -1.0, 1.0)))
        # 방위각은 ±180 경계를 넘으므로 히스토그램의 빈 칸 비율로 커버리지를 셉니다.
        occupied = np.unique(np.floor((az + 180.0) / 2.0).astype(int))  # 2° 빈
        frames.append((len(pts), len(occupied) * 2.0, el.min(), el.max(), d.min(), d.max()))
        allpts.append(pts)

    if not frames:
        print("!! 포인트가 한 프레임도 나오지 않았습니다")
        return 1

    arr = np.array(frames)
    pts = np.vstack(allpts)
    d = np.linalg.norm(pts, axis=1)
    el = np.degrees(np.arcsin(np.clip(pts[:, 2] / d, -1.0, 1.0)))

    if args.hist:
        az = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        hist, edges = np.histogram(az, bins=36, range=(-180, 180))
        print("\n방위각 10° 히스토그램 (전체 프레임 합):")
        for h, e in zip(hist, edges):
            print(f"  {e:+7.0f}° {h:7d} {'#' * int(60 * h / max(hist.max(), 1))}")

    print()
    print(f"프레임 {len(frames)}개 집계  (mode={args.mode}, render={args.render_hz:.0f}Hz)")
    print(f"  프레임당 점 개수     : {arr[:, 0].mean():.0f}  (min {arr[:, 0].min():.0f} / max {arr[:, 0].max():.0f})")
    print(f"  프레임당 방위각 커버 : {arr[:, 1].mean():.1f}°  (min {arr[:, 1].min():.1f}°)")
    print(f"  고도각 범위          : {el.min():.2f}° ~ {el.max():.2f}°")
    print(f"  거리 범위            : {d.min():.2f} ~ {d.max():.2f} m")

    az_ok = arr[:, 1].min() >= 0.95 * EXPECT_AZIMUTH_DEG
    el_ok = abs(el.min() - EXPECT_ELEV_MIN_DEG) < 2.0 and abs(el.max() - EXPECT_ELEV_MAX_DEG) < 3.0
    print()
    print(f"  방위각 360°  : {'PASS' if az_ok else 'FAIL'}")
    print(f"  고도 −7~+52° : {'PASS' if el_ok else 'FAIL'}")
    ok = az_ok and el_ok
    print(f"\n{'PASS — MID-360 프로파일이 적용됐습니다' if ok else 'FAIL — 다른 라이다로 대체됐습니다'}")
    return 0 if ok else 1


try:
    code = main()
finally:
    simulation_app.close()
sys.exit(code)
