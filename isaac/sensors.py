"""G1 기본 장착 센서의 실장 위치 (공식 URDF 그대로).

출처: ``03_IsaacPDW/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf``
     (지금 쓰는 ``g1_29dof_rev_1_0.usd`` 와 동일 리비전)

.. code-block:: xml

    <!-- d435 -->
    <joint name="d435_joint" type="fixed">
      <origin xyz="0.0576235 0.01753 0.42987" rpy="0 0.8307767239493009 0"/>
      <parent link="torso_link"/>

    <!-- mid360 -->
    <joint name="mid360_joint" type="fixed">
      <origin xyz="0.0002835 0.00003 0.41618" rpy="0 0.04014257279586953 0"/>
      <parent link="torso_link"/>

주의: **D435i 는 수평 아래로 47.6° 기울어져 있습니다.**
가슴에 달려 발밑 지형을 보는 배치라 원거리 전방 시야가 제한됩니다.
Nav2 local costmap 의 유효 관측 거리를 여기에 맞춰 잡아야 합니다.

좌표 규약
--------
URDF 링크는 ROS 바디 규약(X 전방, Y 좌, Z 상)이고 USD Camera 는
**-Z 시선, +Y 상, +X 우** 규약입니다. 둘을 잇는 고정 회전이 :data:`Q_ROS_TO_USD_CAMERA` 이며,
검증 결과 ``(w,x,y,z) = (0.5, 0.5, -0.5, -0.5)`` 입니다.
"""

from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- 실장 위치

D435_PARENT = "torso_link"
D435_XYZ = np.array([0.0576235, 0.01753, 0.42987], dtype=np.float64)
D435_RPY = np.array([0.0, 0.8307767239493009, 0.0], dtype=np.float64)
"""D435i RGB-D 카메라. pitch 0.8308 rad = 수평 아래 47.6°."""

MID360_PARENT = "torso_link"
MID360_XYZ = np.array([0.0002835, 0.00003, 0.41618], dtype=np.float64)
MID360_RPY = np.array([0.0, 0.04014257279586953, 0.0], dtype=np.float64)
"""Livox MID-360 3D LiDAR. pitch 0.0401 rad = 아래 2.3° (거의 수평)."""

IMU_TORSO_PARENT = "torso_link"
IMU_TORSO_XYZ = np.array([-0.03959, -0.00224, 0.14792], dtype=np.float64)

IMU_PELVIS_PARENT = "pelvis"
IMU_PELVIS_XYZ = np.array([0.04525, 0.0, -0.08339], dtype=np.float64)
"""정책이 사용하는 IMU 는 pelvis 쪽입니다 (base_ang_vel / projected_gravity 출처)."""

# D435i 기본 사양 (실기 매핑 시 1280x720 을 썼으나 시뮬은 부하를 낮춰 시작)
D435_SIM_RESOLUTION = (640, 480)
D435_SIM_FPS = 15
D435_HFOV_DEG = 69.4  # RGB 센서 수평 FOV
D435_REAL_RESOLUTION = (1280, 720)  # 기존 rtabmap.db 가 사용한 해상도

# --------------------------------------------------------------------------- 쿼터니언

Q_ROS_TO_USD_CAMERA = np.array([0.5, 0.5, -0.5, -0.5], dtype=np.float64)
"""ROS 바디 규약 → USD Camera 규약 고정 회전 (w, x, y, z).

카메라 축을 바디 프레임으로 쓰면 camX=−bodyY, camY=+bodyZ, camZ=−bodyX 이고,
이 회전행렬(det=1, 정규직교 확인)을 쿼터니언으로 바꾼 값입니다.
"""


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis RPY → 쿼터니언 (w, x, y, z)."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """쿼터니언 곱 (w, x, y, z)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def d435_link_quat() -> np.ndarray:
    """d435_link 자세 (torso_link 기준, ROS 규약)."""
    return rpy_to_quat(*D435_RPY)


def d435_camera_quat() -> np.ndarray:
    """USD Camera prim 에 넣을 자세 (torso_link 기준).

    d435_link 회전에 ROS→USD 카메라 축 변환을 합성한 값입니다.
    """
    return quat_mul(d435_link_quat(), Q_ROS_TO_USD_CAMERA)


def mid360_link_quat() -> np.ndarray:
    return rpy_to_quat(*MID360_RPY)


def setup_sensor_frames(robot_prim_path: str, add_camera: bool = True, verify: bool = True):
    """G1 USD 에 이미 존재하는 센서 프레임을 확인하고 카메라 prim 만 추가합니다.

    ``g1_29dof_rev_1_0.usd`` 에는 `d435_link`, `mid360_link`, `imu_in_torso`,
    `imu_in_pelvis` 가 **URDF 와 동일한 변환으로 이미 들어 있습니다.**
    따라서 새로 만들지 않고 기존 prim 을 그대로 씁니다. 카메라 렌더링을 하려면
    USD ``Camera`` prim 이 별도로 필요하므로 그것만 `d435_link` 의 자식으로 붙입니다.
    (부모가 이미 정확한 위치·자세를 잡아주므로 카메라의 로컬 변환은 **축 보정 회전뿐**입니다.)

    Args:
        robot_prim_path: 예 ``"/World/G1"``
        add_camera: USD Camera prim 생성 여부
        verify: 기존 prim 의 변환이 URDF 값과 일치하는지 검사

    Returns:
        prim 경로 딕셔너리
    """
    # Isaac Sim 런타임에서만 임포트 가능하므로 함수 안에서 임포트합니다.
    from isaacsim.core.utils.prims import define_prim, get_prim_at_path
    from pxr import Gf, UsdGeom

    torso = f"{robot_prim_path}/{D435_PARENT}"
    paths = {
        "d435_link": f"{torso}/d435_link",
        "mid360_link": f"{torso}/mid360_link",
        "imu_in_torso": f"{torso}/imu_in_torso",
        "imu_in_pelvis": f"{robot_prim_path}/{IMU_PELVIS_PARENT}/imu_in_pelvis",
    }

    expected = {
        "d435_link": (D435_XYZ, d435_link_quat()),
        "mid360_link": (MID360_XYZ, mid360_link_quat()),
        "imu_in_torso": (IMU_TORSO_XYZ, np.array([1.0, 0.0, 0.0, 0.0])),
        "imu_in_pelvis": (IMU_PELVIS_XYZ, np.array([1.0, 0.0, 0.0, 0.0])),
    }

    for key, path in paths.items():
        prim = get_prim_at_path(path)
        if not prim.IsValid():
            raise RuntimeError(
                f"USD 에 센서 프레임이 없습니다: {path}\n"
                f"다른 리비전의 G1 USD 를 쓰고 있을 수 있습니다."
            )
        if not verify:
            continue
        xf = UsdGeom.Xformable(prim)
        ops = {o.GetOpName(): o.Get() for o in xf.GetOrderedXformOps()}
        want_xyz, want_quat = expected[key]
        got_xyz = np.array(ops.get("xformOp:translate", (0, 0, 0)), dtype=np.float64)
        if not np.allclose(got_xyz, want_xyz, atol=1e-6):
            raise RuntimeError(f"{key} 위치 불일치: USD={got_xyz}, URDF={want_xyz}")
        q = ops.get("xformOp:orient")
        if q is not None:
            im = q.GetImaginary()
            got_quat = np.array([q.GetReal(), im[0], im[1], im[2]], dtype=np.float64)
            # 쿼터니언은 부호 반전이 같은 회전이므로 절댓값 비교
            if not (np.allclose(got_quat, want_quat, atol=1e-5) or np.allclose(-got_quat, want_quat, atol=1e-5)):
                raise RuntimeError(f"{key} 자세 불일치: USD={got_quat}, URDF={want_quat}")

    if add_camera:
        # d435_link 가 이미 위치·자세를 잡아주므로 축 보정 회전만 적용합니다.
        paths["camera"] = f"{paths['d435_link']}/d435_camera"
        cam_prim = define_prim(paths["camera"], "Camera")
        xf = UsdGeom.Xformable(cam_prim)
        xf.ClearXformOpOrder()
        w, x, y, z = (float(v) for v in Q_ROS_TO_USD_CAMERA)
        xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

        cam = UsdGeom.Camera(cam_prim)
        # D435i RGB: HFOV 69.4° → 기본 aperture 20.955mm 기준 초점거리 환산
        aperture = 20.955
        focal = aperture / (2.0 * np.tan(np.radians(D435_HFOV_DEG) / 2.0))
        cam.GetFocalLengthAttr().Set(float(focal))
        cam.GetHorizontalApertureAttr().Set(float(aperture))
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 20.0))

    return paths
