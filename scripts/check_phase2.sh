#!/usr/bin/env bash
# Phase 2 검증 — 프로세스 B(시스템 ROS 2)에서 실행합니다.
#
#   터미널 A:  source scripts/isaac_env.sh && python isaac/g1_nav_sim.py --headless
#   터미널 B:  bash scripts/check_phase2.sh
#
# Isaac Sim(py3.11)이 발행하는 토픽을 시스템 ROS 2(py3.10)에서 받을 수 있는지,
# TF 트리가 온전한지, /cmd_vel 로 실제로 걷는지를 확인합니다.
# `set -u` 금지: ROS setup.bash 가 미정의 변수를 참조해 source 시 죽습니다.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/ros_env.sh"

fail=0
step() { echo; echo "── $* ────────────────────────────────"; }

step "1. 토픽 목록"
topics=$(ros2 topic list 2>/dev/null)
echo "$topics"
for t in /clock /odom /tf /cmd_vel; do
  if grep -qx "$t" <<<"$topics"; then
    echo "   OK   $t"
  else
    echo "   없음 $t"; fail=1
  fi
done

step "2. 발행 주기 (5초 측정)"
for t in /clock /odom; do
  echo "-- $t"
  timeout 6 ros2 topic hz "$t" 2>&1 | head -3
done

step "3. TF 트리"
timeout 10 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8

step "4. odom 메시지 한 건"
timeout 5 ros2 topic echo /odom --once 2>&1 | head -25

step "5. /cmd_vel 로 전진 명령 (8초)"
echo "   vx=0.5 인가 — Isaac 창에서 걷는지 확인하세요"
timeout 8 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 >/dev/null 2>&1
echo "   명령 종료 — 워치독(0.5s)이 로봇을 세워야 합니다"

step "6. 명령 후 odom 위치"
timeout 5 ros2 topic echo /odom --once --field pose.pose.position 2>&1 | head -6

echo
if [ "$fail" -eq 0 ]; then
  echo "결과: 토픽 확인 통과. 위 2~6 출력을 눈으로 검증하세요."
else
  echo "결과: 누락된 토픽이 있습니다. 터미널 A 의 Isaac Sim 이 떠 있는지,"
  echo "      양쪽 ROS_DOMAIN_ID / RMW_IMPLEMENTATION / ROS_LOCALHOST_ONLY 가 같은지 확인하세요."
fi
exit "$fail"
