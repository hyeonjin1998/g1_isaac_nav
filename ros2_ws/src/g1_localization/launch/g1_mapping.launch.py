"""RTAB-Map 매핑 (Phase 3) — 프로세스 B 에서 실행.

기존 `~/Downloads/rtabmap.db` 의 파라미터 세트를 승계해 시뮬 씬의 새 DB 를 만듭니다.
실기 DB 와 설정을 맞춰야 나중에 그대로 갈아끼울 수 있습니다.

사용법::

    # 터미널 A (Isaac)
    source scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --scene full_warehouse --camera --lidar

    # 터미널 B (ROS 2)
    source scripts/ros_env.sh
    ros2 launch g1_localization g1_mapping.launch.py

    # 터미널 C — 로봇을 몰고 다니며 맵 작성
    source scripts/ros_env.sh
    python scripts/drive_patrol.py

    # 저장은 rtabmap 종료 시 자동 (database_path)

기존 실기 DB 에서 읽어온 설정 (rtabmap 0.23.7)::

    Reg/Strategy=0(Visual)  Reg/Force3DoF=false  Kp/DetectorStrategy=8(GFTT/ORB)
    Grid/3D=true  Grid/CellSize=0.05  Optimizer/Strategy=2(GTSAM)
    Rtabmap/DetectionRate=1  Vis/MinInliers=20  Mem/IncrementalMemory=true
"""

import os

# 맵 DB 는 저장소에 커밋되지 않습니다(수백 MB~GB). 저장소를 다른 경로에 클론했다면
# G1_ARCH_ROOT 를 설정하세요 — scripts/ros_env.sh 가 자동으로 export 합니다.
_ARCH_ROOT = os.environ.get("G1_ARCH_ROOT", os.path.expanduser("~/Project/05_Arch"))
_DEFAULT_DB = os.path.join(_ARCH_ROOT, "ros2_ws/src/g1_localization/maps/sim_warehouse.db")

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 실기 DB 와 동일하게 유지할 핵심 파라미터.
# 값을 바꾸면 나중에 실기 DB 로 갈아끼울 때 거동이 달라집니다.
RTABMAP_PARAMS = {
    # --- 등록 / 최적화 ---
    # **실기 DB 의 Reg/Strategy=0(Visual) 을 그대로 쓰면 안 됩니다.**
    # 실측(explored.db, 2026-08-13): 수락된 루프 클로저 15 개 중 13 개가 오검출이었고
    # 병진 오차가 3~18 m 였습니다. `/odom` 이 PhysX 참값이므로 이 오차는 전부 오검출입니다.
    # 원인은 반복 패턴 창고 + **바닥만 보는 카메라**(D435i 는 아래 47.6°)라 시각 특징이
    # 서로 구별되지 않는 것입니다. 오검출이 그래프에 박히면 이후 RTAB-Map 이
    # `RGBD/OptimizeMaxError` 초과를 감지해 **모든 클로저를 거부**하고(81 회 관측)
    # 맵은 뒤틀린 채로 영영 복구되지 않습니다.
    # 2=VisIcp: 시각 정합으로 초기값을 잡고 라이다 ICP 로 검증합니다.
    # 18 m 어긋난 가설은 대응점이 잡히지 않아 `Icp/CorrespondenceRatio` 에서 탈락합니다.
    "Reg/Strategy": "2",  # 0=Visual, 1=ICP, 2=Visual+ICP 검증
    # 평지 창고 + 보행 로봇이므로 z/roll/pitch 를 그래프에서 제외합니다.
    # (6DoF 로 두면 골반 진동이 그래프 오차로 누적돼 클로저 각도 오차를 키웠습니다.)
    "Reg/Force3DoF": "true",
    "Kp/DetectorStrategy": "8",  # GFTT/ORB
    # --- 평행 통로 오검출 차단 ---
    # 실측(explored.db, 3460 노드 — 이미 Reg/Strategy=2 + ICP 검증이 켜진 상태):
    # 수락 클로저 23 개 중 12 개가 오검출, **전부 병진 오차 14.85 m** 였습니다.
    # 참값으로 두 지점을 찍으면 y 는 같고 x 만 14.8~15.1 m 차이 — 창고의 **평행 통로
    # 간격**입니다. 두 번째 통로에 들어서는 순간 맵이 첫 통로 위로 접혀 얹힙니다.
    #
    # ICP 검증도, `RGBD/OptimizeMaxError` 도 못 막습니다. 평행 통로는 3D 형상까지 같아
    # ICP 가 깨끗하게 정합되고, 오검출 12 개가 전부 같은 오프셋으로 **서로 일관**되어
    # 그래프에 모순이 안 생깁니다. 검증 단계가 아니라 **후보 생성 단계**를 막아야 합니다.
    #
    # 그래서 외형 기반 루프 클로저 가설을 통째로 기각시킵니다 (가설값은 확률이라 1 을
    # 넘을 수 없으므로 2.0 은 "절대 성립 안 함"입니다). 남는 클로저는
    # `RGBD/ProximityBySpace`(현재 위치 반경 `RGBD/LocalRadius`=10 m + ICP 검증)뿐이라
    # 15 m 떨어진 통로는 후보에 오르지 못합니다.
    "Rtabmap/LoopThr": "2.0",
    #
    # **`Kp/MaxFeatures` 를 음수로 두면 안 됩니다.** 한때 -1(추출 안 함)로 막았는데,
    # RTAB-Map 은 시그니처의 특징을 **정합에도 재사용**하기 때문에 `Reg/Strategy=2` 의
    # 시각 정합까지 같이 죽었습니다. 실측: 1418 노드 맵에서 루프 클로저 0 개,
    # 시각 어휘 0 개 — **그 맵으로는 재측위가 원리적으로 불가능**했습니다.
    # (복구는 `rtabmap-reprocess --Kp/MaxFeatures 500 --Rtabmap/LoopThr 2.0` 으로
    #  저장된 RGB 에서 어휘만 다시 뽑으면 되고, 지오메트리는 mm 단위로 보존됐습니다.)
    #
    # 기본값 500 을 유지해 어휘를 남깁니다. 위의 LoopThr 이 오검출을 막으므로
    # 어휘가 있어도 매핑 중 통로 접힘은 일어나지 않습니다
    # (실측: 어휘 502,756 개를 만들면서 외형 클로저 0 개, 근접 클로저 7 개 전부 정상).
    "Kp/MaxFeatures": "500",
    # 20 → 35: 오검출 클로저들이 20 인라이어 문턱을 그대로 통과했습니다.
    "Vis/MinInliers": "35",
    "Optimizer/Strategy": "2",  # GTSAM
    "RGBD/ProximityBySpace": "true",
    # **매핑 모드에서는 true.** 그래프 최적화의 고정점을 첫 노드(맵 원점)가 아니라
    # **마지막 노드(로봇 현재 위치)** 로 잡습니다.
    #
    # false 였을 때 실측: 맵이 커지며 전역 재조립이 일어날 때마다 로봇의 map 좌표가
    # 통째로 이동해 `map→odom` 이 튀었습니다 (12회, 최대 9.4 m). 그러면 Nav2 의
    # 전역 경로가 로봇 실제 위치와 어긋나 "앞이 비었는데 경로를 못 따르고 멈추는"
    # 증상이 납니다 (전방점유 0% 로 확인 — 장애물 문제가 아니었음).
    # 재조립 시각과 `Maps update` 급증(0.003s → 0.17~0.23s) 시각이 일치했습니다.
    #
    # true 면 로봇 주변 좌표가 고정되고 대신 맵 내용이 이동합니다. 경로 추종에는
    # 이쪽이 유리합니다. 단, 맵 전체 좌표계가 시간에 따라 이동하므로 **측위 모드
    # (g1_localization.launch.py)에서는 false 를 유지**합니다.
    "RGBD/OptimizeFromGraphEnd": "true",
    "Rtabmap/DetectionRate": "1",
    "Mem/IncrementalMemory": "true",  # 매핑 모드
    "Mem/BinDataKept": "true",
    # --- 점유격자 ---
    # 실기 DB 는 Grid/Sensor=1(카메라)이었으나 D435i 가 아래 47.6° 를 향해
    # 원거리 전방을 못 봅니다. MID-360 을 함께 써서 격자 품질을 올립니다.
    "Grid/Sensor": "2",  # 0=lidar, 1=camera, 2=both
    "Grid/3D": "true",
    "Grid/CellSize": "0.05",
    "Grid/RangeMax": "8.0",  # 실기 DB 는 5.0 (카메라 전용). 라이다 사용으로 상향
    "Grid/RayTracing": "true",  # 자유공간을 채워 Nav2 가 통로를 인식하게 함
    "Grid/MaxGroundHeight": "0.15",
    "Grid/MaxObstacleHeight": "1.8",
    "GridGlobal/OccupancyThr": "0.5",
    "GridGlobal/ProbHit": "0.7",
    "GridGlobal/ProbMiss": "0.4",
    # --- 보행 로봇 대응 / 루프 클로저 검증 ---
    # pelvis 가 상하 ±3cm, roll/pitch ±5° 진동하므로 ICP 보정 창을 넉넉히.
    "Icp/VoxelSize": "0.05",
    # 0.1 → 0.3: 클로저 검증용 초기값(시각 정합)이 수십 cm 어긋나 있어도 정상 클로저는
    # 수렴해야 합니다. 0.1 은 정상 클로저까지 같이 떨어뜨립니다.
    "Icp/MaxCorrespondenceDistance": "0.3",
    "Icp/PointToPlane": "true",
    # **오검출 클로저를 실제로 걸러내는 문턱입니다.**
    # 기본 0.1 은 10% 만 대응돼도 통과시킵니다. 엉뚱한 장소끼리는 0.3 을 못 넘깁니다.
    "Icp/CorrespondenceRatio": "0.3",
    # ICP 가 되돌릴 수 있는 보정량 상한 (m / rad). 초기값이 크게 틀리면 즉시 탈락.
    "Icp/MaxTranslation": "1.0",
    "Icp/MaxRotation": "0.5",
}


def generate_launch_description() -> LaunchDescription:
    db_path = LaunchConfiguration("database_path")
    use_lidar = LaunchConfiguration("use_lidar")
    delete_db = LaunchConfiguration("delete_db")

    common = {
        "use_sim_time": True,
        # base_footprint 를 로봇 기준 프레임으로 씁니다 (보행 진동 배제).
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan_cloud": use_lidar,
        "subscribe_odom_info": False,
        # Isaac 의 센서 스탬프가 완전히 동기되지 않으므로 근사 동기화가 필요합니다.
        "approx_sync": True,
        "sync_queue_size": 30,
        "topic_queue_size": 30,
        "qos_image": 1,
        "qos_camera_info": 1,
        "qos_scan_cloud": 1,
        "qos_odom": 1,
        "wait_for_transform": 0.5,
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
                description="생성할 RTAB-Map DB 경로",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="MID-360 포인트클라우드를 격자 생성에 사용",
            ),
            DeclareLaunchArgument(
                "viz",
                default_value="false",
                description="rtabmap_viz GUI 실행 (헤드리스 환경에서는 false)",
            ),
            DeclareLaunchArgument(
                "delete_db",
                default_value="false",
                description=(
                    "true 면 기존 DB 를 지우고 처음부터 매핑합니다. "
                    "**기본 false** — 실수로 완성된 맵을 날리는 사고가 있었습니다."
                ),
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[common, {"database_path": db_path}, RTABMAP_PARAMS],
                remappings=remappings,
                # -d 는 기존 DB 를 **삭제**합니다. 실수로 완성된 맵을 날린 적이 있어
                # 기본값을 false 로 두고 명시적으로 요청할 때만 붙입니다.
                arguments=[PythonExpression(["'-d' if '", delete_db, "'=='true' else ''"])],
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
