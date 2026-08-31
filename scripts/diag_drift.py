#!/usr/bin/env python3
"""측위 점프 / 유령 장애물 진단 — "후반부에 경로를 못 따르고, 앞이 비었는데 고착" 원인 추적.

두 증상이 같은 원인을 가리킵니다:

    후반부로 갈수록 전역 경로를 못 따름   ← map→odom 이 튀면 경로와 로봇 위치가 어긋남
    앞에 아무것도 없는데 고착             ← 점프 이전에 찍힌 장애물이 국소 코스트맵에 잔류

이 노드는 추측 없이 세 가지를 기록합니다:

1. **map→odom 점프량** — 측위 보정이 얼마나 자주, 얼마나 크게 튀는지
2. **로봇 주변 국소 코스트맵 점유율** — 앞이 비었는데 고비용인지
3. **전역 경로와 로봇의 이격** — 경로를 못 따르는 정도

사용법::

    source scripts/ros_env.sh
    python3 scripts/diag_drift.py            # 화면 출력
    python3 scripts/diag_drift.py --csv /tmp/drift.csv
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

JUMP_WARN = 0.20
"""이 이상 map→odom 이 한 번에 변하면 경고. 경로 추종이 깨지는 수준입니다."""


class DriftDiag(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("diag_drift")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args

        self.buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.buf, self)

        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._local: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_local, latched)
        self._plan: Path | None = None
        self.create_subscription(Path, "/plan", self._on_plan, 5)

        self._prev_mo: tuple[float, float, float] | None = None
        self._jumps: list[float] = []
        self._csv = open(args.csv, "w") if args.csv else None
        if self._csv:
            self._csv.write("t,jump_m,front_occ_pct,path_dev_m\n")

        self.create_timer(args.period, self._tick)
        self.get_logger().info("측위 점프 / 유령 장애물 진단 시작")

    def _on_local(self, m: OccupancyGrid) -> None:
        self._local = m

    def _on_plan(self, m: Path) -> None:
        self._plan = m

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, a: str, b: str):
        try:
            t = self.buf.lookup_transform(a, b, rclpy.time.Time())
            r = t.transform.rotation
            yaw = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:  # noqa: BLE001
            return None

    def _front_occupancy(self) -> float | None:
        """로봇 전방 0.3~1.5 m, 좌우 ±0.5 m 구간의 고비용 셀 비율 [%]."""
        g = self._local
        p = self._lookup("odom", "base_footprint")
        if g is None or p is None:
            return None
        i = g.info
        rx, ry, yaw = p
        hits = tot = 0
        f = 0.3
        while f <= 1.5:
            lat = -0.5
            while lat <= 0.5:
                x = rx + f * math.cos(yaw) - lat * math.sin(yaw)
                y = ry + f * math.sin(yaw) + lat * math.cos(yaw)
                cx = int((x - i.origin.position.x) / i.resolution)
                cy = int((y - i.origin.position.y) / i.resolution)
                if 0 <= cx < i.width and 0 <= cy < i.height:
                    tot += 1
                    if g.data[cy * i.width + cx] >= 90:
                        hits += 1
                lat += 0.1
            f += 0.1
        return 100.0 * hits / tot if tot else None

    def _path_deviation(self) -> float | None:
        """로봇에서 전역 경로까지의 최단 거리 [m]. 크면 경로를 못 따르는 것."""
        p = self._lookup("map", "base_footprint")
        if p is None or self._plan is None or not self._plan.poses:
            return None
        rx, ry, _ = p
        return min(
            math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry) for ps in self._plan.poses
        )

    def _tick(self) -> None:
        mo = self._lookup("map", "odom")
        if mo is None:
            return
        jump = 0.0
        if self._prev_mo is not None:
            jump = math.hypot(mo[0] - self._prev_mo[0], mo[1] - self._prev_mo[1])
        self._prev_mo = mo

        front = self._front_occupancy()
        dev = self._path_deviation()

        if jump >= JUMP_WARN:
            self._jumps.append(jump)
            self.get_logger().warn(
                f"map→odom 점프 {jump:.3f} m — 전역 경로가 로봇 위치와 어긋납니다 "
                f"(누적 {len(self._jumps)}회, 최대 {max(self._jumps):.3f} m)"
            )

        self.get_logger().info(
            f"점프={jump:.3f}m  전방점유={('--' if front is None else f'{front:.0f}%')}  "
            f"경로이격={('--' if dev is None else f'{dev:.2f}m')}"
        )
        if self._csv:
            self._csv.write(
                f"{self._now():.2f},{jump:.4f},"
                f"{-1 if front is None else front:.1f},{-1 if dev is None else dev:.3f}\n"
            )
            self._csv.flush()

    def summary(self) -> None:
        if self._jumps:
            self.get_logger().warn(
                f"요약: {JUMP_WARN} m 이상 점프 {len(self._jumps)}회, "
                f"최대 {max(self._jumps):.3f} m, 평균 {sum(self._jumps)/len(self._jumps):.3f} m"
            )
        else:
            self.get_logger().info(f"요약: {JUMP_WARN} m 이상의 측위 점프 없음")
        if self._csv:
            self._csv.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--period", type=float, default=1.0, help="샘플 주기 [s, sim]")
    parser.add_argument("--csv", default="", help="CSV 저장 경로")
    args = parser.parse_args()

    rclpy.init()
    node = DriftDiag(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
