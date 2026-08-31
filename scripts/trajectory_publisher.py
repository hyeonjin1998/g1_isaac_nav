#!/usr/bin/env python3
"""로봇이 실제로 지나간 궤적을 `nav_msgs/Path` 로 발행 — RViz 가시화용.

Nav2 는 **계획 경로**(`/plan`)만 발행하고 실제 주행 궤적은 남기지 않습니다.
"계획대로 갔는지"를 눈으로 비교하려면 실주행 궤적이 따로 필요합니다.

    /plan            초록  Nav2 전역 계획
    /local_plan      파랑  RPP 가 실제로 추종 중인 국소 궤적
    /traveled_path   빨강  ← 이 노드가 발행하는 실제 지나간 경로

`map` 프레임 기준으로 기록하므로 측위 보정이 반영됩니다.
(`map` 이 없으면 `odom` 으로 자동 폴백합니다.)

사용법::

    source scripts/ros_env.sh
    python3 scripts/trajectory_publisher.py
    python3 scripts/trajectory_publisher.py --reset   # 기록 초기화 후 시작
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

MIN_STEP = 0.05
"""이 거리 이상 움직였을 때만 점을 추가합니다 (보행 진동으로 점이 뭉치는 것 방지)."""

MAX_POINTS = 5000


class TrajectoryPublisher(Node):
    def __init__(self, frame: str) -> None:
        super().__init__("trajectory_publisher")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.pub = self.create_publisher(Path, "/traveled_path", 10)
        self.buf = tf2_ros.Buffer()
        # 리스너를 변수에 보관하지 않으면 GC 되어 TF 가 채워지지 않습니다.
        self._listener = tf2_ros.TransformListener(self.buf, self)

        self.preferred_frame = frame
        self.path = Path()
        self.path.header.frame_id = frame
        self.create_timer(0.2, self._tick)
        self.get_logger().info(f"/traveled_path 발행 시작 (기준 프레임: {frame})")

    def _lookup(self):
        """map 우선, 없으면 odom 으로 폴백."""
        for frame in (self.preferred_frame, "odom"):
            try:
                t = self.buf.lookup_transform(frame, "base_footprint", rclpy.time.Time())
                return frame, t
            except Exception:  # noqa: BLE001
                continue
        return None, None

    def _tick(self) -> None:
        frame, t = self._lookup()
        if t is None:
            return
        if frame != self.path.header.frame_id:
            # 프레임이 바뀌면(측위 확보/상실) 이전 궤적은 좌표계가 달라 의미가 없습니다.
            self.get_logger().warn(f"기준 프레임 변경 {self.path.header.frame_id} → {frame}, 궤적 초기화")
            self.path.poses.clear()
            self.path.header.frame_id = frame

        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = t.header.stamp
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = 0.0
        p.pose.orientation = t.transform.rotation

        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.hypot(p.pose.position.x - last.x, p.pose.position.y - last.y) < MIN_STEP:
                return
        self.path.poses.append(p)
        if len(self.path.poses) > MAX_POINTS:
            del self.path.poses[0]

        self.path.header.stamp = t.header.stamp
        self.pub.publish(self.path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", default="map", help="기준 프레임 (없으면 odom 폴백)")
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryPublisher(args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
