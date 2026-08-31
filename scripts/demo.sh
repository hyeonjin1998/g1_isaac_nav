#!/usr/bin/env bash
# 현재까지 구축된 것을 눈으로 확인하는 데모.
#
#   bash scripts/demo.sh            # 저장된 맵으로 측위 + RViz + 순찰 주행
#   bash scripts/demo.sh mapping    # 맵을 새로 만들며 관찰
#   bash scripts/demo.sh nav        # 저장된 맵으로 자율주행 — RViz 의 2D Goal Pose 로 목표 지정
#   bash scripts/demo.sh guided     # **맵을 만들면서** 2D Goal Pose 로 직접 몰기 (권장)
#   bash scripts/demo.sh explore    # 자율 탐사 — 창고 전체를 스스로 돌며 맵 완성
#   bash scripts/demo.sh manual     # 직접 /cmd_vel 로 조종
#
# 씬 변경 (기본 full_warehouse 는 공터가 넓습니다):
#   SCENE=warehouse    bash scripts/demo.sh explore   # 선반 적은 창고 (중간)
#   SCENE=simple_room  bash scripts/demo.sh explore   # 작은 실내 (가장 작음)
#   SCENE=office       bash scripts/demo.sh explore   # 사무실 (통로 많음)
#
# **각 컴포넌트 로그는 파일로 분리됩니다.**
# rtabmap 이 1초마다 INFO 를 쏟아내 다른 메시지를 전부 덮어버려서, 어느 단계에서
# 막혔는지 알 수 없었기 때문입니다. 터미널에는 진행 단계와 오류만 남깁니다.
#
# 주의: `set -u` 를 쓰면 안 됩니다.
# ROS 2 의 /opt/ros/humble/setup.bash 는 미정의 변수를 참조하므로
# `set -u` 상태에서 source 하면 스크립트가 종료코드 127 로 죽습니다.
# (그러면 trap cleanup 이 방금 띄운 Isaac Sim 까지 같이 종료시킵니다.)
set -o pipefail

MODE="${1:-localization}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

LOGDIR="/tmp/g1_demo_$(date +%H%M%S)"
mkdir -p "$LOGDIR"

# 이 패턴에 걸리는 프로세스는 이 스택의 구성요소입니다 (시작 전 정리 / 종료 시 정리).
STACK_PAT='[g]1_nav_sim|[r]tabmap_slam|[n]av2_|[b]t_navigator|[c]ontroller_server|[p]lanner_server|[b]ehavior_server|[v]elocity_smoother|[l]ifecycle_manager|[r]viz2 -d|[e]xplore_frontier|[t]rajectory_publisher|[s]tuck_escape|[d]iag_drift|[d]rive_patrol'

kill_stack() {
  ps -eo pid,args | grep -E "$STACK_PAT" | awk '{print $1}' | xargs -r kill -9 2>/dev/null || true
}

PIDS=()
cleanup() {
  echo
  echo "== 종료 중 =="
  # rtabmap 은 SIGINT 로 끝내야 DB 가 저장됩니다.
  pkill -INT -f "rtabmap_slam/rtabmap" 2>/dev/null || true
  sleep 3
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 1
  # 유령 노드가 남으면 다음 실행에서 노드 이름이 충돌해 조용히 오작동합니다.
  kill_stack
  echo "정리 완료.  로그: ${LOGDIR}"
}
trap cleanup EXIT INT TERM

fail() {
  echo
  echo "!! $*"
  echo "   로그를 확인하세요: ${LOGDIR}"
  exit 1
}

# 이전 실행의 유령 정리 — 남아 있으면 노드 이름이 충돌합니다.
stale=$(ps -eo pid,args | grep -E "$STACK_PAT" | awk '{print $1}' | tr '\n' ' ')
if [ -n "${stale// /}" ]; then
  echo "!! 이전 실행 프로세스가 남아 있어 정리합니다: ${stale}"
  kill_stack
  sleep 2
fi

echo "모드: ${MODE}      씬: ${SCENE:-full_warehouse}"
echo "로그: ${LOGDIR}"
echo

# ---------------------------------------------------------------- 1. Isaac Sim
echo "== 1/5  Isaac Sim 시작 =="
echo "   창고 에셋 로딩에 1~2분 걸립니다"
(
  # shellcheck disable=SC1091
  source scripts/isaac_env.sh >/dev/null 2>&1
  export PYTHONUNBUFFERED=1
  python isaac/g1_nav_sim.py --scene "${SCENE:-full_warehouse}" --camera --lidar --duration 0
) >"${LOGDIR}/isaac.log" 2>&1 &
PIDS+=($!)

# shellcheck disable=SC1091
source scripts/ros_env.sh || fail "ROS 2 환경 로드 실패 — scripts/ros_env.sh 확인"

echo -n "   /clock 대기"
for _ in $(seq 1 180); do
  ros2 topic list 2>/dev/null | grep -qx "/clock" && break
  echo -n "."
  sleep 2
done
echo
ros2 topic list 2>/dev/null | grep -qx "/clock" || fail "Isaac Sim 이 뜨지 않았습니다 (isaac.log)"
echo "   Isaac Sim 준비됨"

# ---------------------------------------------------------------- 2. RTAB-Map
echo
if [ "$MODE" = "mapping" ] || [ "$MODE" = "explore" ] || [ "$MODE" = "guided" ]; then
  # 탐사/매핑은 **기존 맵과 다른 파일**에 씁니다.
  # (예전에 같은 경로 + -d 조합으로 완성된 맵을 통째로 날린 적이 있습니다.)
  case "$MODE" in
    explore) DEFAULT_NAME="explored" ;;
    guided)  DEFAULT_NAME="guided" ;;
    *)       DEFAULT_NAME="mapping_$(date +%H%M%S)" ;;
  esac
  DB="${ROOT}/ros2_ws/src/g1_localization/maps/${MAP_NAME:-$DEFAULT_NAME}.db"
  echo "== 2/5  RTAB-Map 매핑 시작 =="
  echo "   DB: ${DB}"
  echo "   (기존 sim_warehouse.db 는 건드리지 않습니다)"
  # 탐사는 반드시 **매핑 모드**로 돌립니다. 측위 모드로 하면 맵 밖으로 나가는 순간
  # map→odom 이 점프해 목표·경로가 계속 바뀝니다.
  ros2 launch g1_localization g1_mapping.launch.py database_path:="${DB}" \
    delete_db:="${DELETE_DB:-true}" >"${LOGDIR}/rtabmap.log" 2>&1 &
else
  echo "== 2/5  RTAB-Map 측위 시작 =="
  ros2 launch g1_localization g1_localization.launch.py >"${LOGDIR}/rtabmap.log" 2>&1 &
fi
PIDS+=($!)

echo -n "   RTAB-Map 대기"
for _ in $(seq 1 60); do
  grep -q "rtabmap ([0-9]" "${LOGDIR}/rtabmap.log" 2>/dev/null && break
  echo -n "."
  sleep 2
done
echo
grep -q "rtabmap ([0-9]" "${LOGDIR}/rtabmap.log" 2>/dev/null || fail "RTAB-Map 기동 실패 (rtabmap.log)"
echo "   RTAB-Map 준비됨"

# ---------------------------------------------------------------- 3. RViz
echo
echo "== 3/5  RViz + 궤적 기록 시작 =="
# 실제 주행 궤적(/traveled_path)은 Nav2 가 발행하지 않으므로 별도 노드로 기록합니다.
python3 scripts/trajectory_publisher.py >"${LOGDIR}/trajectory.log" 2>&1 &
PIDS+=($!)
# 벽 고착 탈출 감시 — Nav2 의 BackUp 은 주행거리를 검증해서 벽에 눌리면 실패합니다.
# 이 노드는 거리를 따지지 않고 후진+원호로 무조건 빠져나옵니다.
python3 scripts/stuck_escape.py >"${LOGDIR}/escape.log" 2>&1 &
PIDS+=($!)
# 측위 점프 / 유령 장애물 진단 — "후반부에 경로 이탈, 앞이 비었는데 고착" 원인 추적
python3 scripts/diag_drift.py --csv "${LOGDIR}/drift.csv" >"${LOGDIR}/drift.log" 2>&1 &
PIDS+=($!)
rviz2 -d "${ROOT}/ros2_ws/src/g1_bringup/rviz/g1_nav.rviz" --ros-args -p use_sim_time:=true \
  >"${LOGDIR}/rviz.log" 2>&1 &
PIDS+=($!)
sleep 5
echo "   RViz 창이 떠 있어야 합니다"

# ---------------------------------------------------------------- 4. Nav2
if [ "$MODE" = "nav" ] || [ "$MODE" = "explore" ] || [ "$MODE" = "guided" ]; then
  echo
  echo "== 4/5  Nav2 시작 =="
  ros2 launch g1_navigation g1_navigation.launch.py >"${LOGDIR}/nav2.log" 2>&1 &
  PIDS+=($!)
  echo -n "   lifecycle 활성화 대기"
  ok=0
  for _ in $(seq 1 60); do
    # `ros2 action list` 는 데몬 캐시를 읽어 **이전 실행의 액션을 그대로 반환**합니다.
    # (실측: Nav2 를 띄우자마자 즉시 "준비됨" 으로 통과했으나 실제로는 미기동)
    # 로그에서 lifecycle 관리자가 활성화를 마쳤는지 직접 확인합니다.
    if grep -q "Managed nodes are active" "${LOGDIR}/nav2.log" 2>/dev/null; then ok=1; break; fi
    if grep -q "Failed to bring up" "${LOGDIR}/nav2.log" 2>/dev/null; then break; fi
    echo -n "."
    sleep 2
  done
  echo
  [ "$ok" -eq 1 ] || fail "Nav2 가 활성화되지 않았습니다 (nav2.log)"
  echo "   Nav2 준비됨"
fi

# ---------------------------------------------------------------- 5. 모드별 동작
echo
case "$MODE" in
  guided)
    echo "== 5/5  사용자 지정 목표로 매핑 =="
    # nav 모드와의 차이: 저쪽은 **측위 모드**라 저장된 맵을 읽기만 하고 맵이 자라지
    # 않습니다. 이 모드는 **매핑 모드**로 돌므로 목표를 찍어 이동하는 동안 맵이 계속
    # 채워집니다. 부트스트랩(bootstrap_localization.py)이 필요 없습니다 — 매핑 모드는
    # 첫 노드부터 map→odom 을 냅니다.
    echo "   RTAB-Map 이 매핑 모드로 돌고 있어 이동하는 만큼 맵이 자랍니다."
    echo
    echo "   RViz 상단 '2D Goal Pose' 로 가고 싶은 곳을 찍으세요."
    echo "   회색(미탐사) 영역 경계 쪽으로 조금씩 찍어 나가면 됩니다."
    echo "   한 번에 멀리 찍기보다 5~10 m 씩 끊어 찍는 편이 안정적입니다."
    echo
    echo "   Ctrl+C 로 종료하면 맵이 저장됩니다:"
    echo "     ${DB}"
    wait
    ;;
  explore)
    echo "== 5/5  자율 탐사 시작 =="
    echo "   프론티어(자유공간과 미탐사의 경계)를 찾아 스스로 목표를 정합니다."
    echo "   RViz 에서 회색(미탐사) 영역이 줄어드는 것을 보세요."
    echo
    python3 scripts/explore_frontier.py 2>&1 | tee "${LOGDIR}/explore.log"
    echo
    echo "   탐사 종료. Ctrl+C 로 전체를 종료하면 맵이 저장됩니다."
    wait
    ;;
  nav)
    echo "== 5/5  측위 부트스트랩 =="
    # RTAB-Map 은 로봇이 움직여야 map→odom 을 냅니다. Nav2 는 map 이 있어야 계획하므로
    # 여기서 한 번 밀어줘야 순환이 풀립니다.
    python3 scripts/bootstrap_localization.py 2>&1 | tee "${LOGDIR}/bootstrap.log" | tail -3
    echo
    echo "   준비 완료. RViz 상단의 '2D Goal Pose' 로 목표를 찍으세요."
    echo "   Ctrl+C 로 종료"
    wait
    ;;
  manual)
    echo "== 5/5  수동 조종 모드 =="
    echo
    echo "   다른 터미널에서:"
    echo "     source \"$ROOT\"/scripts/ros_env.sh"
    echo "     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5}}\" -r 20"
    echo
    echo "   주의: 보행 개시 임계값 때문에 vx 는 0.5 이상이어야 걷습니다."
    echo "         선회는 제자리 회전 대신 vx=0.4 + angular.z=0.4 원호로 주세요."
    echo
    echo "   Ctrl+C 로 종료"
    wait
    ;;
  *)
    echo "== 5/5  순찰 주행 시작 =="
    echo "   RViz 에서 맵 위를 이동하는 로봇과 포인트클라우드를 보세요"
    python3 scripts/drive_patrol.py --straight 10 --turn 4 --loops 3 2>&1 | tee "${LOGDIR}/patrol.log"
    echo
    echo "   순찰 완료. Ctrl+C 로 종료"
    wait
    ;;
esac
