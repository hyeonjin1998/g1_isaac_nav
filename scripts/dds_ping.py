#!/usr/bin/env python3
"""Phase 0 검증: Isaac Sim 내장 rclpy(cp311) ↔ 시스템 ROS 2(cp310) DDS 상호운용.

이 프로젝트에서 가장 먼저 깨질 수 있는 지점이라 Isaac Sim 을 띄우기 전에 격리 검증합니다.
Isaac Sim 자체는 임포트하지 않습니다 — 순수하게 rclpy 통신만 확인합니다.

사용법
------
터미널 A (Isaac 쪽):
    source <repo>/scripts/isaac_env.sh
    python <repo>/scripts/dds_ping.py pub

터미널 B (시스템 ROS 2 쪽):
    source <repo>/scripts/ros_env.sh
    ros2 topic echo /dds_ping
    # 또는 역방향 검증:
    #   B 에서  ros2 topic pub /dds_pong std_msgs/String "{data: hello}"
    #   A 에서  python dds_ping.py sub
"""

from __future__ import annotations

import sys


def _fail(msg: str) -> None:
    print(f"[dds_ping] FAIL: {msg}", file=sys.stderr)
    raise SystemExit(1)


try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError as exc:  # noqa: BLE001
    _fail(
        f"rclpy 임포트 실패 ({exc}).\n"
        "  isaac_env.sh 를 source 했는지, PYTHONPATH 에 내장 rclpy 가 있는지 확인하세요."
    )


class Pinger(Node):
    def __init__(self) -> None:
        super().__init__("dds_ping_pub")
        self._pub = self.create_publisher(String, "/dds_ping", 10)
        self._n = 0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self._n += 1
        msg = String(data=f"ping {self._n} from python {sys.version_info.major}.{sys.version_info.minor}")
        self._pub.publish(msg)
        self.get_logger().info(f"published: {msg.data}")


class Ponger(Node):
    def __init__(self) -> None:
        super().__init__("dds_ping_sub")
        self.create_subscription(String, "/dds_pong", self._on_msg, 10)
        self.get_logger().info("/dds_pong 대기 중…")

    def _on_msg(self, msg: String) -> None:
        self.get_logger().info(f"received: {msg.data}")


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "pub"
    if mode not in ("pub", "sub"):
        _fail("사용법: dds_ping.py [pub|sub]")

    print(f"[dds_ping] python {sys.version.split()[0]}, rclpy={rclpy.__file__}")
    rclpy.init()
    node = Pinger() if mode == "pub" else Ponger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
