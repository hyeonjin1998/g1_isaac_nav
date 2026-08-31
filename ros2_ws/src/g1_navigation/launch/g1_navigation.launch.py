"""Nav2 (Phase 5) — 프로세스 B 에서 실행.

측위는 RTAB-Map 이 담당하므로 **AMCL 을 띄우지 않습니다.**
맵(`/map`)도 RTAB-Map 이 발행하므로 `map_server` 도 띄우지 않습니다.

사용법::

    # 터미널 A (Isaac)
    source scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --scene full_warehouse --camera --lidar

    # 터미널 B (측위)
    source scripts/ros_env.sh
    ros2 launch g1_localization g1_localization.launch.py

    # 터미널 C (내비게이션)
    source scripts/ros_env.sh
    ros2 launch g1_navigation g1_navigation.launch.py

    # RViz 에서 '2D Goal Pose' 로 목표 지정
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 이 순서대로 lifecycle 전이가 일어납니다.
LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def generate_launch_description() -> LaunchDescription:
    pkg = Path(get_package_share_directory("g1_navigation"))
    default_params = str(pkg / "params" / "g1_nav2_params.yaml")

    bt_xml = str(pkg / "behavior_trees" / "navigate_no_spin.xml")
    bt_xml_poses = str(pkg / "behavior_trees" / "navigate_through_poses_no_spin.xml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    common = {"use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[params_file, common],
                # velocity_smoother 를 거쳐 나가도록 컨트롤러 출력을 우회시킵니다.
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                output="screen",
                parameters=[params_file, common],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                output="screen",
                parameters=[params_file, common],
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                output="screen",
                parameters=[
                    params_file,
                    common,
                    # 두 BT 모두 교체해야 합니다. bt_navigator 는 기동 시 둘 다 로드하며,
                    # 한쪽만 바꾸면 through_poses 경로에서 Spin/BackUp 이 호출됩니다.
                    {
                        "default_nav_to_pose_bt_xml": bt_xml,
                        "default_nav_through_poses_bt_xml": bt_xml_poses,
                    },
                ],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                output="screen",
                parameters=[params_file, common],
                # 최종 출력만 로봇으로 나갑니다.
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    common,
                    {"autostart": True, "node_names": LIFECYCLE_NODES},
                ],
            ),
        ]
    )
