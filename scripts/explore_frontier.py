#!/usr/bin/env python3
"""프론티어 기반 자율 탐사 — 창고 전체를 스스로 돌며 맵을 완성합니다.

왜 매핑 모드에서 돌리는가
----------------------
측위(localization) 모드로 탐사하면 로봇이 맵 밖으로 나가는 순간 시각 매칭이 깨지고
`map→odom` 보정이 점프합니다. 그러면 Nav2 가 보는 로봇 위치가 실제와 어긋나 목표와
경로가 계속 바뀌고, 로봇이 엉뚱한 곳으로 갑니다 (실측: 25 m 주행 후 맵 밖 x=−25).

탐사는 **매핑 모드**(`Mem/IncrementalMemory=true`)에서 돌립니다. 맵이 계속 자라므로
"맵 밖" 이라는 상태 자체가 없고, 위 문제가 성립하지 않습니다.

프론티어란
--------
**자유공간(0)과 맞닿은 미탐사(−1) 경계**입니다. 그쪽으로 가면 새 영역이 보입니다.
프론티어가 없어지면 탐사 완료입니다.

이 로봇의 제약 반영 (Phase 1 실측)
--------------------------------
- 제자리 회전 불가 → 프론티어에서 "둘러보기" 를 못 하므로, 접근 자체로 시야를 넓힙니다
- 반경 0.30 m + 여유 → 목표는 주변이 충분히 트인 곳만 선택
- 최소 이동 거리 → 너무 가까운 프론티어는 건너뜁니다 (보행 개시 임계값 때문)

사용법::

    # 터미널 A: 시뮬
    source scripts/isaac_env.sh
    python isaac/g1_nav_sim.py --scene full_warehouse --camera --lidar

    # 터미널 B: 매핑 (측위 모드 아님!)
    source scripts/ros_env.sh
    ros2 launch g1_localization g1_mapping.launch.py \\
        database_path:=/tmp/explored.db

    # 터미널 C: Nav2
    ros2 launch g1_navigation g1_navigation.launch.py

    # 터미널 D: 탐사
    python3 scripts/explore_frontier.py
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import deque

import rclpy
import tf2_ros
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

UNKNOWN = -1
FREE_MAX = 20
"""이 값 이하를 자유공간으로 봅니다 (점유격자는 0=자유, 100=점유)."""

OCCUPIED_MIN = 65


class FrontierExplorer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("frontier_explorer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args

        latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)

        self.buf = tf2_ros.Buffer()
        # 리스너를 변수에 보관하지 않으면 GC 되어 TF 가 채워지지 않습니다.
        self._listener = tf2_ros.TransformListener(self.buf, self)

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 도달 가능성 사전 검사용. 로봇을 움직이지 않고 플래너에게 경로만 물어봅니다.
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.blacklist: list[tuple[float, float]] = []
        self.visited: list[tuple[float, float]] = []

    # ------------------------------------------------------------------ 입력

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def wait_clock(self) -> None:
        """use_sim_time 노드는 /clock 수신 전까지 now()==0 입니다.
        이를 기다리지 않고 시간을 재면 첫 /clock 도착 즉시 타임아웃이 만료됩니다."""
        t0 = self.get_clock().now().nanoseconds
        while rclpy.ok() and self.get_clock().now().nanoseconds == t0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_xy(self, retries: int = 50) -> tuple[float, float] | None:
        for _ in range(retries):
            try:
                t = self.buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                return t.transform.translation.x, t.transform.translation.y
            except Exception:  # noqa: BLE001
                rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def wait_map(self, timeout: float = 30.0) -> bool:
        t0 = self.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._map is not None:
                return True
        return False

    # ------------------------------------------------------------------ 프론티어

    def find_frontiers(self) -> list[tuple[float, float, int]]:
        """(x, y, 크기) 목록을 반환합니다. 큰 군집일수록 새로 보이는 영역이 넓습니다."""
        g = self._map
        assert g is not None
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = g.data

        def idx(cx: int, cy: int) -> int:
            return cy * w + cx

        # 1) 프론티어 셀: 자유공간이면서 4-이웃에 미탐사가 있는 셀
        is_frontier = bytearray(w * h)
        for cy in range(1, h - 1):
            base = cy * w
            for cx in range(1, w - 1):
                i = base + cx
                v = data[i]
                if v < 0 or v > FREE_MAX:
                    continue
                if (
                    data[i - 1] == UNKNOWN
                    or data[i + 1] == UNKNOWN
                    or data[i - w] == UNKNOWN
                    or data[i + w] == UNKNOWN
                ):
                    is_frontier[i] = 1

        # 2) 군집화 (8-이웃 BFS)
        seen = bytearray(w * h)
        clusters: list[tuple[float, float, int]] = []
        for cy in range(1, h - 1):
            for cx in range(1, w - 1):
                i = idx(cx, cy)
                if not is_frontier[i] or seen[i]:
                    continue
                q = deque([(cx, cy)])
                seen[i] = 1
                cells = []
                while q:
                    x, y = q.popleft()
                    cells.append((x, y))
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            nx, ny = x + dx, y + dy
                            if not (0 < nx < w - 1 and 0 < ny < h - 1):
                                continue
                            j = idx(nx, ny)
                            if is_frontier[j] and not seen[j]:
                                seen[j] = 1
                                q.append((nx, ny))
                if len(cells) < self.args.min_cluster:
                    continue
                mx = sum(c[0] for c in cells) / len(cells)
                my = sum(c[1] for c in cells) / len(cells)
                clusters.append((ox + mx * res, oy + my * res, len(cells)))
        return clusters

    def has_clearance(self, x: float, y: float) -> bool:
        """목표 주변이 충분히 트여 있는지. 로봇 반경 0.30 m 안에 장애물이 있으면
        플래너가 목표를 거부합니다 (실측으로 확인)."""
        g = self._map
        assert g is not None
        i = g.info
        r = self.args.clearance
        steps = max(2, int(r / i.resolution))
        for dx in range(-steps, steps + 1):
            for dy in range(-steps, steps + 1):
                if dx * dx + dy * dy > steps * steps:
                    continue
                cx = int((x - i.origin.position.x) / i.resolution) + dx
                cy = int((y - i.origin.position.y) / i.resolution) + dy
                if not (0 <= cx < i.width and 0 <= cy < i.height):
                    return False
                v = g.data[cy * i.width + cx]
                if v >= OCCUPIED_MIN:
                    return False
        return True

    def pick_goal(self, rx: float, ry: float) -> tuple[float, float, int] | None:
        """가까우면서 큰 프론티어를 고릅니다."""
        best = None
        best_score = -1e18
        for fx, fy, size in self.find_frontiers():
            d = math.hypot(fx - rx, fy - ry)
            # 너무 가까우면 보행 개시 임계값 때문에 제대로 못 갑니다.
            if d < self.args.min_dist:
                continue
            if any(math.hypot(fx - bx, fy - by) < self.args.blacklist_radius for bx, by in self.blacklist):
                continue
            if not self.has_clearance(fx, fy):
                continue
            # 가까울수록·클수록 좋음. 거리에 가중치를 더 둡니다(왕복 비용).
            score = size * self.args.size_weight - d
            if score > best_score:
                best_score, best = score, (fx, fy, size)
        return best

    def reachable(self, x: float, y: float, timeout: float = 8.0) -> bool:
        """플래너에게 경로가 있는지만 물어봅니다 (로봇은 움직이지 않음).

        프론티어 탐지는 "자유공간과 미탐사의 경계" 를 찾을 뿐 **거기까지 갈 수 있는지는
        모릅니다.** 창고 벽 바깥은 영원히 미탐사로 남으므로 벽에 붙은 프론티어가 계속
        후보로 올라오고, 그때마다 실제 주행을 시도하다 실패해 20초씩 낭비합니다
        (실측: 6회 연속 실패, 매번 ~21초).

        사전 검사로 걸러내면 즉시 다음 후보로 넘어갑니다.
        """
        if not self.planner.server_is_ready() and not self.planner.wait_for_server(timeout_sec=5):
            # 플래너 액션이 없으면 검사를 건너뜁니다 (기존 동작 유지).
            return True

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False

        fut = self.planner.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if not res.done() or res.result() is None:
            return False
        r = res.result()
        return r.status == 4 and len(r.result.path.poses) > 0

    # ------------------------------------------------------------------ 주행

    def goto(self, x: float, y: float, timeout: float = 120.0) -> int | None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # 최종 방위는 제어할 수 없으므로(제자리 회전 불가) 의미 없는 값입니다.
        goal.pose.pose.orientation.w = 1.0

        # 액션 서버가 실제로 살아 있는지 매번 확인합니다.
        # (ros2 action list 는 데몬 캐시를 읽어 죽은 서버를 살아 있다고 보고합니다.)
        if not self.nav.server_is_ready() and not self.nav.wait_for_server(timeout_sec=10):
            self.get_logger().error("navigate_to_pose 서버가 응답하지 않습니다 — Nav2 상태 확인")
            return None

        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        gh = fut.result()
        if gh is None:
            self.get_logger().warn("   목표 전송 응답 없음 (액션 서버 무응답)")
            return None
        if not gh.accepted:
            self.get_logger().warn("   목표가 거부됨 (도달 불가 위치일 수 있음)")
            return None
        res = gh.get_result_async()
        t0 = self.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds * 1e-9 - t0 > timeout:
                gh.cancel_goal_async()
                return None
        return res.result().status

    def run(self) -> int:
        self.wait_clock()
        self.get_logger().info("/clock 수신")
        if not self.wait_map():
            self.get_logger().error("/map 을 받지 못했습니다. 매핑 노드가 떠 있는지 확인하세요.")
            return 1
        if not self.nav.wait_for_server(timeout_sec=30):
            self.get_logger().error("navigate_to_pose 액션 서버 없음. Nav2 가 떠 있는지 확인하세요.")
            return 1

        for it in range(self.args.max_goals):
            rclpy.spin_once(self, timeout_sec=0.5)
            rxy = self.robot_xy()
            if rxy is None:
                self.get_logger().error("map→base_footprint 조회 실패 — 측위/TF 확인")
                return 1
            rx, ry = rxy

            target = self.pick_goal(rx, ry)
            if target is None:
                # 시작 직후나 목표 실패 직후에는 맵이 아직 얇아 프론티어가 안 보일 수 있습니다.
                # 곧바로 "완료" 로 끝내면 로봇이 한 발짝도 못 뗀 채 종료됩니다 (실측).
                # 맵이 자랄 시간을 주고 여러 번 재확인합니다.
                self.get_logger().info("프론티어를 찾지 못함 — 맵 갱신을 기다립니다…")
                found = False
                for retry in range(self.args.empty_retries):
                    t0 = self.get_clock().now().nanoseconds * 1e-9
                    while self.get_clock().now().nanoseconds * 1e-9 - t0 < 3.0:
                        rclpy.spin_once(self, timeout_sec=0.2)
                    target = self.pick_goal(rx, ry)
                    if target is not None:
                        self.get_logger().info(f"   재확인 {retry + 1}회차에 발견")
                        found = True
                        break
                if not found:
                    self.get_logger().info(
                        f"남은 프론티어 없음 → 탐사 완료 ({it}개 목표 수행, "
                        f"블랙리스트 {len(self.blacklist)}개)"
                    )
                    return 0

            fx, fy, size = target
            d = math.hypot(fx - rx, fy - ry)
            self.get_logger().info(
                f"[{it + 1}/{self.args.max_goals}] 프론티어 (%.2f, %.2f) 크기 {size}셀, 거리 %.1f m"
                % (fx, fy, d)
            )

            if self.args.precheck and not self.reachable(fx, fy):
                self.get_logger().info("   경로 없음(벽 너머 등) → 즉시 제외")
                self.blacklist.append((fx, fy))
                continue

            status = self.goto(fx, fy, self.args.goal_timeout)
            if status == 4:
                self.get_logger().info("   도달")
                self.visited.append((fx, fy))
            else:
                self.get_logger().warn(f"   실패(status={status}) → 블랙리스트 등록")
                self.blacklist.append((fx, fy))

        self.get_logger().info(f"최대 목표 수({self.args.max_goals}) 도달 — 종료")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--max-goals", type=int, default=40)
    parser.add_argument("--goal-timeout", type=float, default=120.0, help="목표당 제한 시간 [s, sim]")
    parser.add_argument("--min-dist", type=float, default=1.5, help="이보다 가까운 프론티어는 무시 [m]")
    parser.add_argument("--min-cluster", type=int, default=12, help="최소 군집 크기 [셀]")
    parser.add_argument("--clearance", type=float, default=0.45, help="목표 주변 확보 반경 [m]")
    parser.add_argument("--blacklist-radius", type=float, default=1.2, help="실패 지점 재시도 금지 반경 [m]")
    parser.add_argument("--size-weight", type=float, default=0.05, help="군집 크기 가중치")
    parser.add_argument(
        "--no-precheck",
        dest="precheck",
        action="store_false",
        help="도달 가능성 사전 검사를 끕니다 (기본 켜짐)",
    )
    parser.add_argument(
        "--empty-retries",
        type=int,
        default=8,
        help="프론티어를 못 찾았을 때 맵 갱신을 기다리며 재확인할 횟수",
    )
    args = parser.parse_args()

    rclpy.init()
    node = FrontierExplorer(args)
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("중단 요청")
        code = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
