# 프로세스 A (Isaac Sim, Python 3.11) 환경
#
#   source <repo>/scripts/isaac_env.sh
#
# !! /opt/ros/humble/setup.bash 를 절대 source 하지 마세요.
#    시스템 ROS 2 는 Python 3.10 용이라 venv(3.11)의 rclpy 임포트를 깨뜨립니다.
#    대신 Isaac Sim 이 번들한 cp311 빌드 rclpy 를 사용합니다.

if [ -n "${ROS_DISTRO:-}" ] && [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  echo "[isaac_env] 경고: 시스템 ROS 2가 이미 source된 셸입니다 (ROS_DISTRO=${ROS_DISTRO})."
  echo "[isaac_env]       새 터미널에서 다시 시도하세요."
fi

# Isaac Sim(pip) 이 설치된 venv. 경로가 다르면 ISAAC_VENV 로 지정하세요.
#   export ISAAC_VENV=~/isaacsim_venv
ISAAC_VENV="${ISAAC_VENV:-$HOME/IsaacLab/env_isaaclab}"
if [ ! -f "${ISAAC_VENV}/bin/activate" ]; then
  echo "[isaac_env] 오류: venv 를 찾을 수 없습니다: ${ISAAC_VENV}"
  echo "[isaac_env]       ISAAC_VENV 로 Isaac Sim venv 경로를 지정하세요 (README '1. Isaac Sim')."
  return 1 2>/dev/null || exit 1
fi
source "${ISAAC_VENV}/bin/activate"

_SP=$(python -c 'import site; print(site.getsitepackages()[0])')
export ISAAC_ROS_BRIDGE="${_SP}/isaacsim/exts/isaacsim.ros2.bridge/humble"

if [ ! -d "${ISAAC_ROS_BRIDGE}/rclpy" ]; then
  echo "[isaac_env] 오류: 내장 rclpy를 찾을 수 없습니다: ${ISAAC_ROS_BRIDGE}/rclpy"
  return 1 2>/dev/null || exit 1
fi

export LD_LIBRARY_PATH="${ISAAC_ROS_BRIDGE}/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${ISAAC_ROS_BRIDGE}/rclpy:${PYTHONPATH:-}"

# 프로세스 B 와 반드시 일치해야 하는 값
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1

unset _SP
echo "[isaac_env] ready — python=$(python -V 2>&1), rmw=${RMW_IMPLEMENTATION}, domain=${ROS_DOMAIN_ID}"
