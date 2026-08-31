#!/usr/bin/env python3
"""측위 부트스트랩 — Nav2 로 목표를 주기 전에 실행합니다.

왜 필요한가
----------
RTAB-Map 은 측위 모드에서 **연속된 정확한 측위를 확인한 뒤에야** `map→odom` TF 를 냅니다::

    [ WARN] Rtabmap.cpp:3882::process() Localization was good, but waiting for
            another one to be more accurate (RGBD/MaxOdomCacheSize>0)

로봇이 정지해 있으면 두 번째 측위가 잡히지 않아 `map` 프레임이 영영 생기지 않습니다.
그런데 Nav2 는 `map` 프레임이 있어야 계획을 세울 수 있으니 **닭-달걀 상황**이 됩니다.

이 스크립트는 `map→base_footprint` 가 잡힐 때까지 로봇을 조금씩 전진시켜 그 고리를 끊습니다.
(RGBD/MaxOdomCacheSize=0 으로 즉시 발행시킬 수도 있지만, 그러면 첫 측위의 오차가
 그대로 남습니다. Phase 4 실측 정확도 0.118 m 를 유지하려고 이 방식을 택했습니다.)

사용법::

    source scripts/ros_env.sh
    python3 scripts/bootstrap_localization.py
"""

from __future__ import annotations

import argparse
import sys

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist

# Phase 1 실측: vx 0.3 은 무반응, 0.5 에서 추종 오차 1.4%.
NUDGE_VX = 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=60.0, help="포기까지 시간 [s, sim]")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("bootstrap_localization")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    def now() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    # /clock 수신 대기 — sim time 이 0 에 멈춰 있으면 타임아웃 계산이 무의미합니다.
    node.get_logger().info("/clock 대기 중…")
    t0 = now()
    while rclpy.ok() and now() == t0:
        rclpy.spin_once(node, timeout_sec=0.1)

    def have_map() -> bool:
        return buf.can_transform(args.map_frame, args.base_frame, rclpy.time.Time())

    if have_map():
        node.get_logger().info(f"{args.map_frame} 프레임이 이미 있습니다. 부트스트랩 불필요.")
        _shutdown(node)
        return 0

    node.get_logger().info(f"{args.map_frame} 프레임이 없습니다. 전진하며 측위를 유도합니다.")
    msg = Twist()
    msg.linear.x = NUDGE_VX

    start = now()
    while rclpy.ok() and now() - start < args.timeout:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        if have_map():
            elapsed = now() - start
            node.get_logger().info(f"측위 성립 ({elapsed:.1f}s 만에 {args.map_frame} 프레임 확인)")
            _stop(node, pub)
            _shutdown(node)
            return 0

    node.get_logger().error(
        f"{args.timeout:.0f}s 안에 {args.map_frame} 프레임이 생기지 않았습니다.\n"
        "  점검: 1) RTAB-Map 이 이 DB 로 측위 중인지  2) 로봇 시작 위치가 맵 범위 안인지\n"
        "        3) 카메라/라이다 토픽이 살아 있는지"
    )
    _stop(node, pub)
    _shutdown(node)
    return 1


def _stop(node, pub) -> None:
    for _ in range(10):
        pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)


def _shutdown(node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
