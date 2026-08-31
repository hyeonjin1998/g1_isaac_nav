#!/usr/bin/env python3
"""매핑용 순찰 주행 — 프로세스 B(시스템 ROS 2)에서 실행.

RTAB-Map 으로 맵을 만들려면 로봇을 씬 곳곳으로 몰고 다니며 루프 클로저를 만들어야 합니다.
수동 텔레옵보다 재현 가능하도록 스크립트로 고정합니다.

**MID-360 기반 반응형 회피 내장** (`--reactive`, 기본 켜짐)
--------------------------------------------------------
시간표만 따르는 개루프 순찰은 벽에 막혀 제자리에서 밀기만 합니다. 실측으로 두 번 겪었습니다:

    직진 8초(≈4m)  → (5.14, 3.30) 에서 정지, 명령은 계속 인가됨
    직진 12초(≈6m) → (5.30, −4.97) 에서 정지, RTAB-Map 노드가 79 에서 멈춤

로봇이 멈추면 RTAB-Map 은 새 노드를 만들지 않으므로 **맵이 그 자리에서 성장을 멈춥니다.**
그래서 전방 라이다 거리를 보고 막히면 직진을 중단하고 선회로 전환합니다.

정책 특성 반영 (Phase 1 실측)
---------------------------
- 보행 개시 임계값이 있어 **작은 명령은 무시**됩니다 → 전진 0.5, 선회는 전진과 함께 인가
- 제자리 회전(wz 단독)은 1.0 에서야 겨우 돌고 38% 오차 → **원호 선회만 사용**
- 후진은 추종이 부정확 → 사용하지 않음

사용법::

    source scripts/ros_env.sh
    python3 scripts/drive_patrol.py                     # 반응형 사각 순찰
    python3 scripts/drive_patrol.py --loops 4           # 4바퀴
    python3 scripts/drive_patrol.py --no-reactive       # 개루프 (권장하지 않음)
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

FORWARD_VX = 0.5
"""전진 속도. 임계값(0.3 실패 / 0.5 동작) 위이면서 추종 오차가 1.4% 로 가장 좋은 값."""

TURN_VX = 0.4
TURN_WZ = 0.4
"""원호 선회. 이 조합은 양축 모두 ~10% 오차로 추종됩니다 (wz 단독은 무반응)."""

BLOCK_DIST = 1.8
"""이 거리 안에 전방 장애물이 있으면 직진을 중단합니다.

정지거리 여유: 전진 0.5 m/s, 감속 한계를 고려하면 1.5 m 면 충분하지만
보행 로봇은 즉시 멈추지 못하므로 여유를 둡니다.
"""

CONE_HALF_WIDTH = 0.5
"""전방 검사 폭(±). 로봇 반경 0.30 m + 여유."""


class PatrolDriver(Node):
    def __init__(self, straight_s: float, turn_s: float, loops: int, reactive: bool) -> None:
        super().__init__("g1_patrol_driver")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.straight_s = straight_s
        self.turn_s = turn_s
        self.loops = loops
        self.reactive = reactive
        self._front_dist = float("inf")
        self._cloud_seen = False

        if reactive:
            # QoS 는 Isaac 쪽 writer 와 맞춥니다 (best effort 로도 수신되도록 기본값 사용).
            self.create_subscription(PointCloud2, "/mid360/points", self._on_cloud, 5)

    # ------------------------------------------------------------------ 센싱

    def _on_cloud(self, msg: PointCloud2) -> None:
        """전방 원뿔 안의 최소 거리를 갱신합니다.

        포인트는 mid360_link 프레임입니다. 이 프레임은 base_link 대비
        2.3° 만 기울어져 있고 원점이 (0, 0, 0.46) 이므로, 전방 판정에는
        TF 변환 없이 x/y 를 그대로 써도 무방합니다.
        지면(라이다 기준 z ≈ −1.24 m)은 제외합니다.
        """
        best = float("inf")
        for x, y, z in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            if x <= 0.25 or abs(y) > CONE_HALF_WIDTH:
                continue
            if z < -0.9:  # 지면 및 발밑 반사 제외
                continue
            d = math.hypot(x, y)
            if d < best:
                best = d
        self._front_dist = best
        self._cloud_seen = True

    @property
    def blocked(self) -> bool:
        return self.reactive and self._front_dist < BLOCK_DIST

    # ------------------------------------------------------------------ 주행

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _drive(self, vx: float, wz: float, duration_s: float, label: str, abort_if_blocked: bool) -> str:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.get_logger().info(f"{label}: vx={vx:.2f} wz={wz:.2f} for {duration_s:.1f}s (sim)")

        end = self._now() + duration_s
        while rclpy.ok() and self._now() < end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if abort_if_blocked and self.blocked:
                self.get_logger().info(f"   전방 {self._front_dist:.2f} m 막힘 → 선회로 전환")
                return "blocked"
        return "done"

    def stop(self) -> None:
        # 워치독(0.5s)이 있지만 명시적으로 0 을 보내 즉시 정지시킵니다.
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().info("정지 명령 전송")

    def run(self) -> None:
        self.get_logger().info("/clock 대기 중…")
        t0 = self._now()
        while rclpy.ok() and self._now() == t0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"sim time 수신 (t={self._now():.2f})")

        if self.reactive:
            self.get_logger().info("/mid360/points 대기 중…")
            for _ in range(200):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._cloud_seen:
                    break
            if not self._cloud_seen:
                self.get_logger().warn("라이다 수신 실패 — 개루프로 진행합니다")
                self.reactive = False
            else:
                self.get_logger().info(f"라이다 수신 확인 (전방 {self._front_dist:.2f} m)")

        for lap in range(self.loops):
            self.get_logger().info(f"--- {lap + 1}/{self.loops} 바퀴 ---")
            for side in range(4):
                self._drive(FORWARD_VX, 0.0, self.straight_s, f"직진 {side + 1}/4", True)
                # 막혀서 꺾는 경우 조금 더 돌려 확실히 벗어나게 합니다.
                extra = 1.5 if self.blocked else 0.0
                self._drive(TURN_VX, TURN_WZ, self.turn_s + extra, f"선회 {side + 1}/4", False)
        self.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--straight", type=float, default=10.0, help="직진 구간 최대 [s, sim]")
    parser.add_argument("--turn", type=float, default=4.0, help="선회 구간 [s, sim]")
    parser.add_argument("--loops", type=int, default=3, help="순찰 바퀴 수")
    parser.add_argument(
        "--no-reactive",
        dest="reactive",
        action="store_false",
        help="라이다 회피 없이 시간표만 따름 (벽에 막힐 수 있음)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PatrolDriver(args.straight, args.turn, args.loops, args.reactive)
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
