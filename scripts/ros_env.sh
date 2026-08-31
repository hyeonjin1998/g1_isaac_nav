# 프로세스 B (ROS 2 Humble, Python 3.10) 환경
#
#   source <repo>/scripts/ros_env.sh
#
# Nav2 / RTAB-Map / RViz 를 띄우는 터미널에서 사용합니다.
# 이 셸에서는 Isaac Sim venv 를 activate 하지 마세요.

if [ -n "${VIRTUAL_ENV:-}" ]; then
  echo "[ros_env] 경고: Python venv(${VIRTUAL_ENV})가 활성화된 셸입니다. 새 터미널을 권장합니다."
fi

# miniconda 의 python3 가 PATH 앞에 있으면 ROS 2 python 노드가 시스템 python3.10 대신
# conda python 을 잡아 임포트가 깨집니다. PATH 에서 conda 를 뒤로 밀어냅니다.
if command -v python3 >/dev/null && [[ "$(command -v python3)" == *"conda"* ]]; then
  echo "[ros_env] conda python3 감지 → PATH 에서 제외합니다: $(command -v python3)"
  _CONDA_BIN=$(dirname "$(command -v python3)")
  PATH=$(echo "$PATH" | tr ':' '\n' | grep -vFx "${_CONDA_BIN}" | paste -sd:)
  export PATH
  unset _CONDA_BIN
fi

source /opt/ros/humble/setup.bash

# 워크스페이스가 빌드되어 있으면 오버레이
# 저장소 루트. launch 파일이 맵 DB 기본 경로를 잡는 데 씁니다.
export G1_ARCH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_WS="${G1_ARCH_ROOT}/ros2_ws"
if [ -f "${_WS}/install/setup.bash" ]; then
  source "${_WS}/install/setup.bash"
fi
unset _WS

# 프로세스 A 와 반드시 일치해야 하는 값
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

echo "[ros_env] ready — ROS_DISTRO=${ROS_DISTRO}, rmw=${RMW_IMPLEMENTATION}, domain=${ROS_DOMAIN_ID}"
