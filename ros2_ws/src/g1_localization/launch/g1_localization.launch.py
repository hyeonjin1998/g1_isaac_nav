"""RTAB-Map 측위 (Phase 4) — 프로세스 B 에서 실행.

Phase 3 에서 만든 DB 를 읽기 전용으로 열어 `map→odom` 보정을 발행합니다.
매핑 launch 와 **파라미터 세트가 동일**하고 `Mem/IncrementalMemory` 만 false 입니다.
(측위 시 파라미터가 달라지면 DB 의 특징점 기술자와 매칭이 어긋납니다.)

사용법::

    # 터미널 A (Isaac)
    source scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --scene full_warehouse --camera --lidar

    # 터미널 B (ROS 2)
    source scripts/ros_env.sh
    ros2 launch g1_localization g1_localization.launch.py

    # 실기 DB 로 교체할 때
    ros2 launch g1_localization g1_localization.launch.py \\
        database_path:=~/Downloads/rtabmap.db use_lidar:=false

검증
----
로봇을 임의 위치에 재배치한 뒤 수 초 내 재측위되는지, `map→base_footprint` 오차가
0.15 m 이내인지 확인합니다.
"""

import os

# 맵 DB 는 저장소에 커밋되지 않습니다(수백 MB~GB). 저장소를 다른 경로에 클론했다면
# G1_ARCH_ROOT 를 설정하세요 — scripts/ros_env.sh 가 자동으로 export 합니다.
_ARCH_ROOT = os.environ.get("G1_ARCH_ROOT", os.path.expanduser("~/Project/05_Arch"))
_DEFAULT_DB = os.path.join(_ARCH_ROOT, "ros2_ws/src/g1_localization/maps/sim_warehouse.db")

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 매핑과 동일 세트 + 측위 전용 항목.
# 매핑 launch 의 RTABMAP_PARAMS 를 복사하지 않고 여기서 다시 선언하는 이유는
# launch 파일 간 임포트가 설치 경로에 따라 깨지기 쉬워서입니다.
LOCALIZATION_PARAMS = {
    # --- 매핑과 반드시 동일하게 유지 (근거는 g1_mapping.launch.py 주석 참조) ---
    "Reg/Strategy": "2",  # Visual + ICP 검증. 순수 Visual 은 오검출률 13/15 였음
    "Reg/Force3DoF": "true",
    "Kp/DetectorStrategy": "8",
    "Vis/MinInliers": "35",
    "Optimizer/Strategy": "2",
    "RGBD/ProximityBySpace": "true",
    "RGBD/OptimizeFromGraphEnd": "false",
    "Rtabmap/DetectionRate": "1",
    "Grid/Sensor": "2",
    "Grid/3D": "true",
    "Grid/CellSize": "0.05",
    "Grid/RangeMax": "8.0",
    "Grid/RayTracing": "true",
    "Grid/MaxGroundHeight": "0.15",
    "Grid/MaxObstacleHeight": "1.8",
    "Icp/VoxelSize": "0.05",
    "Icp/MaxCorrespondenceDistance": "0.3",
    "Icp/PointToPlane": "true",
    "Icp/CorrespondenceRatio": "0.3",
    "Icp/MaxTranslation": "1.0",
    "Icp/MaxRotation": "0.5",
    # --- 측위 전용 ---
    "Mem/IncrementalMemory": "false",  # 읽기 전용: 새 노드를 추가하지 않음
    "Mem/InitWMWithAllNodes": "true",  # DB 전체를 작업 메모리에 올려 어디서든 재측위
    "RGBD/OptimizeMaxError": "3.0",
    # 첫 측위에서 곧바로 map→odom 을 발행합니다.
    #
    # 기본값(>0)이면 RTAB-Map 은 오도메트리 캐시를 모아 **더 정확한 측위가
    # 한 번 더 잡힐 때까지 TF 를 내지 않습니다**:
    #     [WARN] Localization was good, but waiting for another one to be
    #            more accurate (RGBD/MaxOdomCacheSize>0)
    # 로봇이 정지해 있으면 두 번째 측위가 잡히지 않아 map 프레임이 영영 생기지
    # 않고, Nav2 는 map 이 있어야 계획하므로 교착이 발생합니다.
    # 정확도를 약간 포기하는 대신 측위 가용성을 확보합니다
    # (Phase 4 실측 정확도 0.118 m 기준, 손해는 수 cm 수준으로 예상).
    "RGBD/MaxOdomCacheSize": "0",
}


def generate_launch_description() -> LaunchDescription:
    db_path = LaunchConfiguration("database_path")
    use_lidar = LaunchConfiguration("use_lidar")

    common = {
        "use_sim_time": True,
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan_cloud": use_lidar,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "sync_queue_size": 30,
        "topic_queue_size": 30,
        "qos_image": 1,
        "qos_camera_info": 1,
        "qos_scan_cloud": 1,
        "qos_odom": 1,
        "wait_for_transform": 0.5,
        # 측위 모드에서는 맵을 계속 재발행해 Nav2 static layer 가 항상 받도록 합니다.
        "latch": True,
    }

    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_rect_raw"),
        ("scan_cloud", "/mid360/points"),
        ("odom", "/odom"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=_DEFAULT_DB,
                description="읽어들일 RTAB-Map DB. 실기 맵은 ~/Downloads/rtabmap.db",
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("viz", default_value="false"),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[common, {"database_path": db_path}, LOCALIZATION_PARAMS],
                remappings=remappings,
                # -d 를 주지 않습니다 (기존 DB 를 지우면 안 됨)
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                parameters=[common],
                remappings=remappings,
                condition=IfCondition(LaunchConfiguration("viz")),
            ),
        ]
    )
