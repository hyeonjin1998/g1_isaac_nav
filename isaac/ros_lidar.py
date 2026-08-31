"""Livox MID-360 (근사) RTX 라이다 → ROS 2 PointCloud2 발행.

G1 에 기본 장착된 MID-360 을 `mid360_link` 실장 위치에 생성하고 포인트클라우드를
ROS 2 로 내보냅니다. 카메라와 마찬가지로 replicator writer 를 씁니다.

발행 토픽::

    /mid360/points    sensor_msgs/PointCloud2   frame = mid360_link

근사에 대한 정직한 고지
--------------------
실제 MID-360 은 **비반복 로제트(rosette) 스캔**이라 시간이 지날수록 커버리지가 촘촘해집니다.
Isaac Sim 의 RTX lidar 는 rotary 다중빔 모델만 지원하므로 `lidar_configs/Livox_MID360.json`
은 **FOV(360°×59°, −7~+52°)와 포인트레이트(200k pts/s @ 10Hz)를 맞춘 근사**입니다.
커버리지 범위는 동등하지만 포인트 분포 패턴은 다릅니다.
매핑·장애물 검출 용도로는 충분하나, 스캔 패턴 자체에 민감한 알고리즘을 평가할 때는
이 차이를 감안해야 합니다.

주의
----
RTX 라이다는 레이트레이싱 기반이라 **렌더 파이프라인이 돌 때만** 동작합니다.
headless 여도 ``world.step(render=True)`` 가 필요합니다.
"""

from __future__ import annotations

from pathlib import Path

POINTS_TOPIC = "/mid360/points"
MID360_FRAME = "mid360_link"

CONFIG_DIR = Path(__file__).parent / "lidar_configs"
CONFIG_NAME = "Livox_MID360"
CONFIG_JSON = CONFIG_DIR / f"{CONFIG_NAME}.json"


def register_config_folder() -> None:
    """커스텀 라이다 프로파일 폴더를 Isaac Sim 탐색 경로에 추가합니다.

    Isaac Sim 은 `app.sensors.nv.lidar.profileBaseFolder` 설정(문자열 배열)에서
    프로파일 JSON 을 찾습니다. 번들 폴더만 들어 있으므로 우리 폴더를 덧붙입니다.
    **라이다 prim 을 만들기 전에** 호출해야 합니다.

    주의: Isaac Sim 5.x 에서 이 설정은 **legacy 카메라 prim 라이다 전용**입니다
    (`isaacsim.sensors.rtx/config/extension.toml`: "for (deprecated) camera-based
    Lidar"). :class:`MID360Publisher` 가 `force_camera_prim=True` 를 쓰는 이유입니다.
    """
    import carb

    if not CONFIG_JSON.is_file():
        raise RuntimeError(f"라이다 프로파일 JSON 이 없습니다: {CONFIG_JSON}")

    settings = carb.settings.get_settings()
    key = "/app/sensors/nv/lidar/profileBaseFolder"
    folders = list(settings.get(key) or [])
    ours = str(CONFIG_DIR) + "/"
    if ours not in folders:
        folders.append(ours)
        settings.set(key, folders)


class MID360Publisher:
    """MID-360 RTX 라이다 생성 + PointCloud2 발행.

    Args:
        parent_prim_path: `mid360_link` prim 경로. 라이다를 그 자식으로 붙입니다.
                          (부모가 이미 URDF 실장 위치·자세를 잡고 있으므로
                           라이다의 로컬 변환은 항등입니다.)
    """

    def __init__(self, parent_prim_path: str) -> None:
        import omni.kit.commands
        import omni.replicator.core as rep

        register_config_folder()

        self.prim_path = f"{parent_prim_path}/mid360"
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="mid360",
            parent=parent_prim_path,
            config=CONFIG_NAME,
            # **`force_camera_prim=True` 를 빼면 안 됩니다.**
            # Isaac Sim 5.x 의 기본 경로(OmniLidar prim)는 JSON 프로파일을 아예 읽지
            # 않습니다. `IsaacSensorCreateRtxLidar` 가 인식하는 config 는 하드코딩된
            # USD 에셋 목록(HESAI/Ouster/SICK…)뿐이라, 우리 커스텀 이름은 못 찾고
            # **경고 한 줄만 남긴 뒤 기본 라이다로 조용히 대체**됩니다.
            #
            # 실측(explored.db 스캔 디코딩): 그렇게 만들어진 라이다는 한 프레임당
            # 방위각 24° 부채꼴, 고도 −15~+7°, 최근접 4m 였습니다. 360°×59° 인 줄
            # 알고 만든 맵은 550 노드 중 120 개에만 장애물 셀이 있었습니다.
            #
            # JSON 프로파일은 legacy 카메라 prim 경로에만 남아 있습니다
            # (extension.toml: profileBaseFolder = "for (deprecated) camera-based Lidar").
            # 폐지 예고된 경로이므로 Isaac 업그레이드 때 아래 검증이 먼저 깨집니다.
            force_camera_prim=True,
        )
        if not result or prim is None or not prim.IsValid():
            raise RuntimeError(
                f"RTX 라이다 생성 실패 (config={CONFIG_NAME}). "
                f"프로파일 폴더가 등록됐는지 확인: {CONFIG_DIR}"
            )

        # 프로파일이 붙지 않으면 **조용히 다른 라이다가 됩니다.** 반드시 여기서 죽입니다.
        cfg_attr = prim.GetAttribute("sensorModelConfig")
        applied = cfg_attr.Get() if cfg_attr and cfg_attr.IsValid() else None
        if applied != CONFIG_NAME:
            raise RuntimeError(
                f"라이다 프로파일이 적용되지 않았습니다 (sensorModelConfig={applied!r}, "
                f"기대값={CONFIG_NAME!r}). Isaac Sim 이 legacy 카메라 prim 경로를 없앴다면 "
                f"프로파일을 OmniLidar 스키마 속성으로 포팅해야 합니다."
            )
        self.prim = prim

        self._render_product = rep.create.render_product(
            self.prim.GetPath().pathString, [1, 1]
        )
        writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
        writer.initialize(
            frameId=MID360_FRAME,
            nodeNamespace="",
            queueSize=1,
            topicName=POINTS_TOPIC,
        )
        writer.attach([self._render_product.path])
        self._writer = writer

    def describe(self) -> str:
        return f"MID-360(근사) → {POINTS_TOPIC}, frame={MID360_FRAME}, 360°×59°, 200k pts/s @10Hz"

    def detach(self) -> None:
        try:
            self._writer.detach()
        except Exception:  # noqa: BLE001, S110
            pass
