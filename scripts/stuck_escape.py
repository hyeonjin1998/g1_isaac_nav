#!/usr/bin/env python3
"""벽 고착 탈출 감시 노드 — 갇히면 후진 + 원호 선회로 무조건 빠져나옵니다.

왜 Nav2 의 BackUp 으로는 안 되는가
--------------------------------
`nav2_behaviors/BackUp` 은 **주행거리로 목표 후진량을 검증**합니다. 그런데 이 로봇은

- 후진 추종이 부정확하고 (명령 −0.5 → 실측 −0.389, 22% 부족)
- 벽에 눌린 상태면 아예 안 밀립니다

그래서 `Exceeded time allowance` 로 실패하고, 충돌 검사 투영이 국소 코스트맵을 벗어나면
`Pose Goes Off Grid → Collision Ahead` 로도 실패합니다. 파라미터를 조정해도
"얼마나 물러났는지" 를 따지는 한 벽에 눌린 상황에서는 통과하기 어렵습니다.

이 노드는 **거리를 검증하지 않습니다.** 정해진 시간 동안 후진 명령을 내고, 이어서 원호로
방향을 틀 뿐입니다. 실제로 얼마나 물러났는지는 묻지 않으므로 실패할 여지가 없습니다.

고착 판정
--------
**Nav2 가 목표를 수행 중인데(`navigate_to_pose` 액션이 EXECUTING) 로봇이 멈춰 있으면**
갇힌 것으로 봅니다.

판정 기준을 두 번 고쳤습니다:

1. `/cmd_vel` 에 명령이 있는데 안 움직임 → **발동 0회.**
   RPP 는 전방 충돌을 감지하면 명령 자체를 내지 않습니다
   (실측: collision ahead 315건, 그동안 cmd_vel ≈ 0).
2. 순간 속도가 임계값 미만 → **여전히 발동 0회.**
   보행 로봇은 제자리에서도 다리·몸통이 흔들려 순간 속도가 임계값을 넘나듭니다
   (실측: 20초 넘게 정지 상태인데 고착 카운터가 0.2초에서 계속 리셋).

지금은 **일정 시간 동안의 실제 이동 거리**로 판정합니다. 흔들림에 영향받지 않습니다.

탈출 동작 — 단계적 확대 (Phase 1 실측 반영)
-----------------------------------------
같은 동작을 반복하면 같은 자리에 다시 갇히므로, 시도할 때마다 강도를 올립니다.

===== ================================================ ==========================
시도   동작                                             근거
===== ================================================ ==========================
 1     **제자리 회전** wz = ±1.0                        뒤쪽 공간이 필요 없음
 2     후진 vx = −0.5 → 원호 선회                       회전으로 안 되면 물러남
 3+    후진(길게) → 제자리 회전(크게)                   둘 다 최대로
===== ================================================ ==========================

**제자리 회전을 1순위로 둔 이유**: 벽에 코를 박으면 뒤가 막혀 후진이 무의미할 수 있지만,
회전은 그 자리에서 방향만 바꾸면 됩니다.

wz 는 반드시 **1.0** 이어야 합니다. 실측상 0.5/0.6/0.8 은 무반응이고 1.0 에서야
0.617 rad/s 로 돕니다(38% 오차). 주행 제어에는 못 쓰지만 탈출에는 충분합니다
— 90° 회전에 약 2.5초.

탈출 중에는 Nav2 의 명령을 무시하고 이 노드가 `/cmd_vel` 을 직접 씁니다.

사용법::

    source scripts/ros_env.sh
    python3 scripts/stuck_escape.py
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

BACKWARD_VX = -0.5
"""학습 범위 하한. −0.3 은 무반응, −0.4 에서 −0.284, −0.5 에서 −0.389 (실측)."""

TURN_VX = 0.4
TURN_WZ = 0.5
"""원호 선회 (전진하며 방향 전환). 후진 뒤 방향을 트는 데 씁니다."""

SPIN_WZ = 1.0
"""제자리 회전. 실측: 0.5/0.6/0.8 무반응, **1.0 에서 0.617 rad/s**.
정확도는 나쁘지만(38% 오차) 탈출에는 충분하고, 무엇보다 **뒤쪽 공간이 필요 없습니다.**"""


class StuckEscape(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stuck_escape")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.args = args

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # **`/cmd_vel` 에는 velocity_smoother 도 20 Hz 로 씁니다.**
        # 브리지(ros_io._on_cmd_vel)는 마지막에 온 것만 들고 있으므로, 탈출 중에
        # 우리 후진 명령과 스무더의 명령이 번갈아 들어가 정책이 갈팡질팡합니다.
        # (실측 증상: "글로벌 경로는 후진인데 정책에 후진 입력이 안 들어감")
        # → 탈출을 시작할 때 Nav2 목표를 취소해 스무더 입력을 끊습니다.
        #   컨트롤러가 멈추면 스무더는 velocity_timeout(1.0s) 뒤 조용해집니다.
        self._cancel_cli = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        # Nav2 가 목표를 수행 중인지는 액션 상태 토픽으로 판단합니다.
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, 10
        )

        self._navigating = False
        self._meas_speed = 0.0
        # (시각, x, y) 이력 — 이동 거리 기반 판정용. 순간 속도는 보행 흔들림에
        # 취약해 두 번이나 판정에 실패했습니다.
        self._track: list[tuple[float, float, float]] = []
        self._stuck_since: float | None = None
        self._turn_sign = 1.0
        self._turn_wz = TURN_WZ
        self._escapes = 0
        self._level = 1
        self._back_seconds = 0.0
        # 탈출은 상태 기계로 진행합니다 (idle → back → turn → idle).
        self._phase = "idle"
        self._phase_end = 0.0
        # sim time 은 0 에서 시작하므로 `if self._phase_start` 로 판정하면
        # t=0 에 시작한 탈출에서 안전장치가 무력화됩니다. None 센티널을 씁니다.
        self._phase_start: float | None = None
        # 진단용: 판정이 왜 안 걸리는지 추측하지 않고 기록으로 확인합니다.
        # (판정 기준을 두 번 바꿨는데 두 번 다 발동하지 않았습니다.)
        self._diag_last = 0.0
        self._slow_since: float | None = None
        self._min_speed_while_navigating = 1e9

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"고착 감시 시작 — Nav2 주행 중인데 {args.stuck_seconds:.0f}초간 안 움직이면 탈출"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg: GoalStatusArray) -> None:
        self._navigating = any(s.status == GoalStatus.STATUS_EXECUTING for s in msg.status_list)

    def _on_odom(self, msg: Odometry) -> None:
        v = msg.twist.twist
        self._meas_speed = math.hypot(v.linear.x, v.linear.y) + 0.3 * abs(v.angular.z)
        pos = msg.pose.pose.position
        t = self._now()
        self._track.append((t, pos.x, pos.y))
        # 판정 창의 2배만 보관
        cutoff = t - 2.0 * self.args.stuck_seconds
        while len(self._track) > 2 and self._track[0][0] < cutoff:
            self._track.pop(0)

    def _displacement(self, window: float) -> float | None:
        """최근 `window` 초 동안의 최대 이동 거리. 이력이 부족하면 None."""
        if len(self._track) < 2:
            return None
        now = self._track[-1][0]
        pts = [p for p in self._track if now - p[0] <= window]
        if len(pts) < 2 or now - pts[0][0] < window * 0.8:
            return None  # 창을 채울 만큼의 이력이 아직 없음
        cx, cy = pts[-1][1], pts[-1][2]
        return max(math.hypot(x - cx, y - cy) for _, x, y in pts)

    def _diag(self) -> None:
        """주행 중 상태를 주기적으로 남깁니다 — 고착 판정 미발동 원인 추적용."""
        now = self._now()
        if not self._navigating:
            self._min_speed_while_navigating = 1e9
            self._slow_since = None
            return
        self._min_speed_while_navigating = min(self._min_speed_while_navigating, self._meas_speed)
        # "거의 안 움직임" 구간이 얼마나 이어지는지도 별도로 잽니다.
        if self._meas_speed < self.args.diag_slow_speed:
            if self._slow_since is None:
                self._slow_since = now
        else:
            self._slow_since = None

        if now - self._diag_last >= self.args.diag_period:
            self._diag_last = now
            stuck_for = 0.0 if self._stuck_since is None else now - self._stuck_since
            disp = self._displacement(self.args.stuck_seconds)
            self.get_logger().info(
                f"[진단] 주행중 속도={self._meas_speed:.3f} "
                f"{self.args.stuck_seconds:.0f}초 이동거리="
                f"{'--' if disp is None else f'{disp:.3f}m'} "
                f"고착카운터={stuck_for:.1f}s "
                f"(임계 {self.args.move_distance}m/{self.args.stuck_seconds}s)"
            )

    def _tick(self) -> None:
        if self._phase != "idle":
            self._step_escape()
            return
        self._diag()
        disp = self._displacement(self.args.stuck_seconds)
        # 이력이 부족하면 판정을 미룹니다 (기동 직후 오탐 방지).
        moving = disp is None or disp > self.args.move_distance

        if self._navigating and not moving:
            if self._stuck_since is None:
                self._stuck_since = self._now()
            elif self._now() - self._stuck_since >= self.args.stuck_seconds:
                self._start_escape()
        else:
            self._stuck_since = None

    def _publish(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def _cancel_nav_goal(self) -> None:
        """진행 중인 Nav2 목표를 **전부** 취소합니다.

        빈 goal_info(uuid=0, stamp=0)는 액션 규약상 "모든 목표 취소"를 뜻합니다.
        우리는 목표 핸들을 갖고 있지 않으므로 액션 클라이언트 대신 cancel 서비스를
        직접 부릅니다. 응답은 기다리지 않습니다 — 타이머 콜백을 막으면 안 됩니다.

        탐사(explore_frontier)는 status != 4 를 실패로 보고 다음 프론티어로 넘어가므로
        취소돼도 멈추지 않습니다. `guided` 모드에서는 사용자가 목표를 다시 찍어야 합니다
        (고착된 목표를 계속 붙들고 있는 것보다 낫습니다).
        """
        if not self._cancel_cli.service_is_ready():
            return
        self._cancel_cli.call_async(CancelGoal.Request())

    def _start_escape(self) -> None:
        self._escapes += 1
        self._phase_start = self._now()
        self._cancel_nav_goal()
        # 시도할 때마다 강도를 올립니다. 같은 동작을 반복하면 같은 자리에 다시 갇힙니다.
        level = min(self._escapes, 3)
        self._level = level
        if level == 1:
            # 1순위는 제자리 회전 — 뒤가 막혀 있어도 쓸 수 있습니다.
            self._phase = "spin"
            self._phase_end = self._now() + self.args.spin_seconds
            self.get_logger().warn(
                f"고착 감지 → 탈출 #{self._escapes} [1단계] 제자리 회전 "
                f"{self.args.spin_seconds:.1f}초 (wz=±{SPIN_WZ})"
            )
        else:
            back = self.args.back_seconds * (1.5 if level >= 3 else 1.0)
            self._back_seconds = back
            self._phase = "back"
            self._phase_end = self._now() + back
            self.get_logger().warn(
                f"고착 감지 → 탈출 #{self._escapes} [{level}단계] 후진 {back:.1f}초 후 방향 전환"
            )

    def _step_escape(self) -> None:
        """타이머 틱마다 한 단계씩 진행하는 상태 기계.

        **블로킹 루프를 쓰면 안 됩니다.** 처음에는 `while now < end: publish; spin_once()`
        로 구현했는데, 타이머 콜백 안에서 spin_once 를 부르면 단일 스레드 실행기에서
        재진입이 막혀 `/clock` 이 처리되지 않습니다. 그러면 sim time 이 멈춰 루프가
        영원히 끝나지 않고 **후진 명령만 계속 나갔습니다** (실측: 탈출 감지 1회,
        완료 0회, 로봇이 계속 후진).
        """
        now = self._now()

        # 안전장치: 어떤 이유로든(시계 정지 등) 단계가 끝나지 않으면 강제 복귀합니다.
        # 후진 명령이 무한히 나가는 사고를 한 번 냈기에 이중으로 막습니다.
        budget = (
            self.args.back_seconds * 1.5
            + self.args.turn_seconds
            + self.args.spin_seconds
            + 5.0
        )
        if self._phase_start is not None and now - self._phase_start > budget:
            self.get_logger().error(
                f"탈출 단계가 {budget:.0f}초를 넘겨 강제 종료합니다 (현재 단계: {self._phase})"
            )
            self._publish(0.0, 0.0)
            self._phase = "idle"
            self._phase_start = None
            self._stuck_since = None
            return

        if self._phase == "spin":
            if now < self._phase_end:
                self._publish(0.0, SPIN_WZ * self._turn_sign)
                return
            self._turn_sign *= -1.0
            self._finish()
            return

        if self._phase == "back":
            if now < self._phase_end:
                self._publish(BACKWARD_VX, 0.0)
                return
            if self._level >= 3:
                # 3단계: 후진 뒤 제자리에서 크게 돌립니다.
                self._phase = "spin"
                self._phase_end = now + self.args.spin_seconds * 1.5
                self.get_logger().info(
                    f"   제자리 회전 {self.args.spin_seconds * 1.5:.1f}초 "
                    f"(wz={SPIN_WZ * self._turn_sign:+.1f})"
                )
                return
            wz = TURN_WZ * self._turn_sign
            self._turn_sign *= -1.0
            self._turn_wz = wz
            self._phase = "turn"
            self._phase_end = now + self.args.turn_seconds
            self.get_logger().info(f"   원호 선회 {self.args.turn_seconds:.1f}초 (wz={wz:+.1f})")
            return

        if self._phase == "turn":
            if now < self._phase_end:
                self._publish(TURN_VX, self._turn_wz)
                return
            self._finish()

    def _finish(self) -> None:
        self._publish(0.0, 0.0)
        self._phase = "idle"
        self._phase_start = None
        self._stuck_since = None
        # 탈출 직후에는 이동 이력이 옛 위치를 담고 있어 곧바로 재판정되면
        # 또 고착으로 오인됩니다. 이력을 비워 판정을 새로 시작합니다.
        self._track.clear()
        self.get_logger().info("   탈출 동작 완료 — 제어권 반환")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stuck-seconds", type=float, default=4.0, help="이 시간 이상 안 움직이면 고착 판정")
    parser.add_argument("--back-seconds", type=float, default=2.5, help="후진 지속 시간 [s, sim]")
    parser.add_argument("--turn-seconds", type=float, default=3.0, help="원호 선회 시간 [s, sim]")
    parser.add_argument(
        "--spin-seconds",
        type=float,
        default=4.0,
        help="제자리 회전 시간 [s, sim]. 실측 0.617 rad/s 이므로 4초 ≈ 141°",
    )
    parser.add_argument(
        "--move-distance",
        type=float,
        default=0.25,
        help="판정 창 동안 이만큼도 못 움직였으면 고착 [m]",
    )
    parser.add_argument("--diag-period", type=float, default=5.0, help="진단 로그 주기 [s, sim]")
    parser.add_argument("--diag-slow-speed", type=float, default=0.15, help="이 미만을 '저속'으로 기록")
    args = parser.parse_args()

    rclpy.init()
    node = StuckEscape(args)
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
