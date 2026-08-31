"""D435i RGB-D 스트림을 ROS 2 로 발행 (Isaac Sim replicator writer 방식).

이미지처럼 고빈도·고용량 스트림은 Python rclpy 로 끌어오면 프레임률이 무너지므로
Isaac Sim 내부의 ROS 2 브리지 writer 로 직접 내보냅니다.

발행 토픽 (RTAB-Map RGB-D 입력 규약)::

    /camera/color/image_raw           sensor_msgs/Image   rgb8
    /camera/color/camera_info         sensor_msgs/CameraInfo
    /camera/depth/image_rect_raw      sensor_msgs/Image   32FC1  (미터 단위)
    /camera/depth/camera_info         sensor_msgs/CameraInfo

주의
----
writer 는 **렌더 파이프라인이 돌 때만** 동작합니다. headless 라도
``world.step(render=True)`` 로 스텝해야 프레임이 나갑니다.

프레임 규약
----------
이미지의 ``frame_id`` 는 ROS 광학 프레임(``d435_color_optical_frame``: X 우, Y 하, Z 전방)입니다.
바디 규약인 ``d435_link`` 와는 REP-103 표준 회전 rpy=(−π/2, 0, −π/2) 로 연결되며,
이 정적 변환은 :func:`optical_frame_quat` 이 제공합니다.
"""

from __future__ import annotations

import numpy as np

RGB_TOPIC = "/camera/color/image_raw"
RGB_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_TOPIC = "/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/camera/depth/camera_info"
DEPTH_POINTS_TOPIC = "/camera/depth/points"
"""깊이 영상을 PointCloud2 로도 발행합니다.

Nav2 costmap 은 PointCloud2/LaserScan 만 장애물 소스로 받습니다. 그리고 두 센서는
커버 범위가 상호보완적이라 **둘 다 필요합니다**:

===========  ===================================  ==============================
센서         지면 커버                            역할
===========  ===================================  ==============================
MID-360      16 m 이상 (하단 빔 −7°, 높이 1.24m)  벽·기둥 등 원거리/높은 장애물
D435i        0.49 ~ 2.57 m (아래 47.6° 틸트)      **근거리 바닥 장애물**
===========  ===================================  ==============================

라이다만 쓰면 발밑 2 m 이내의 낮은 장애물이 통째로 사각지대가 됩니다.
"""

COLOR_OPTICAL_FRAME = "d435_color_optical_frame"
DEPTH_OPTICAL_FRAME = "d435_depth_optical_frame"
CAMERA_LINK_FRAME = "d435_link"


def optical_frame_quat() -> np.ndarray:
    """``d435_link`` → 광학 프레임 정적 회전 (w, x, y, z).

    REP-103: 광학 프레임은 바디 프레임 대비 rpy = (−π/2, 0, −π/2).
    """
    from sensors import rpy_to_quat

    return rpy_to_quat(-np.pi / 2, 0.0, -np.pi / 2)


class D435Publisher:
    """D435i 컬러/깊이 스트림 발행기.

    Args:
        camera_prim_path: USD Camera prim 경로
        resolution: (width, height)
        frequency: 발행 주기 [Hz]. 렌더 주기보다 크게 잡아도 렌더 주기로 제한됩니다.
    """

    def __init__(
        self,
        camera_prim_path: str,
        resolution: tuple[int, int] = (640, 480),
        frequency: int = 15,
    ) -> None:
        import omni.replicator.core as rep
        import omni.syntheticdata._syntheticdata as sd
        from omni.syntheticdata import SyntheticData

        self.camera_prim_path = camera_prim_path
        self.resolution = resolution

        self._render_product = rep.create.render_product(camera_prim_path, resolution)
        rp_path = self._render_product.path
        self._writers = []

        # --- RGB ---
        rv_rgb = SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        w_rgb = rep.writers.get(f"{rv_rgb}ROS2PublishImage")
        w_rgb.initialize(
            frameId=COLOR_OPTICAL_FRAME,
            nodeNamespace="",
            queueSize=1,
            topicName=RGB_TOPIC,
        )
        w_rgb.attach([rp_path])
        self._writers.append(w_rgb)

        # --- Depth (32FC1, 미터) ---
        rv_depth = SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        w_depth = rep.writers.get(f"{rv_depth}ROS2PublishImage")
        w_depth.initialize(
            frameId=DEPTH_OPTICAL_FRAME,
            nodeNamespace="",
            queueSize=1,
            topicName=DEPTH_TOPIC,
        )
        w_depth.attach([rp_path])
        self._writers.append(w_depth)

        # --- Depth → PointCloud2 (Nav2 costmap 장애물 소스) ---
        w_pc = rep.writers.get(f"{rv_depth}ROS2PublishPointCloud")
        w_pc.initialize(
            frameId=DEPTH_OPTICAL_FRAME,
            nodeNamespace="",
            queueSize=1,
            topicName=DEPTH_POINTS_TOPIC,
        )
        w_pc.attach([rp_path])
        self._writers.append(w_pc)

        # --- CameraInfo (컬러/깊이 각각) ---
        # RTAB-Map 은 image 와 camera_info 의 frame_id·stamp 가 맞아야 하고,
        # **intrinsic(k) 이 비어 있으면 RGB-D 를 아예 처리하지 못합니다.**
        # writer 에 값을 넘기지 않으면 width=height=0, k=[] 인 빈 메시지가 나갑니다.
        from isaacsim.ros2.bridge import read_camera_info

        info, _camera_prim = read_camera_info(render_product_path=rp_path)  # (CameraInfo, Usd.Prim)
        self.camera_info = info
        if int(info.width) == 0 or not list(info.k):
            raise RuntimeError(
                f"카메라 intrinsic 을 읽지 못했습니다 (width={info.width}, k={list(info.k)}). "
                "Camera prim 의 focalLength / horizontalAperture 설정을 확인하세요."
            )

        for topic, frame in ((RGB_INFO_TOPIC, COLOR_OPTICAL_FRAME), (DEPTH_INFO_TOPIC, DEPTH_OPTICAL_FRAME)):
            w_info = rep.writers.get("ROS2PublishCameraInfo")
            w_info.initialize(
                frameId=frame,
                nodeNamespace="",
                queueSize=1,
                topicName=topic,
                width=int(info.width),
                height=int(info.height),
                projectionType="pinhole",
                k=np.array(info.k).reshape(3, 3),
                r=np.array(info.r).reshape(3, 3),
                p=np.array(info.p).reshape(3, 4),
                physicalDistortionModel=info.distortion_model,
                physicalDistortionCoefficients=np.array(info.d, dtype=np.float32),
            )
            w_info.attach([rp_path])
            self._writers.append(w_info)

    @property
    def render_product_path(self) -> str:
        return self._render_product.path

    def describe(self) -> str:
        w, h = self.resolution
        k = list(self.camera_info.k)
        return (
            f"D435i {w}x{h} → {RGB_TOPIC}, {DEPTH_TOPIC} (+ camera_info)\n"
            f"           frame={COLOR_OPTICAL_FRAME}  "
            f"fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}"
        )

    def detach(self) -> None:
        for w in self._writers:
            try:
                w.detach()
            except Exception:  # noqa: BLE001, S110
                pass
        self._writers.clear()
