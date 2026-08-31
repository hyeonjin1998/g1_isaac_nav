#!/usr/bin/env bash
# ROS 2 Humble + Nav2 + RTAB-Map 설치 (Ubuntu 22.04 jammy)
#
# sudo 비밀번호 입력이 필요하므로 Claude Code 세션이 아닌 *일반 터미널*에서 실행하세요:
#     bash <repo>/scripts/install_ros2.sh
#
# 저장소 등록 절차는 docs.ros.org/en/humble Ubuntu-Install-Debs 기준 (2026-08 확인).
# 구버전 안내에 나오는 ros-archive-keyring.gpg 방식은 키 만료로 더 이상 동작하지 않습니다.
set -euo pipefail

echo "==> [1/5] 로케일 (UTF-8) 확인"
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> [2/5] universe 저장소 활성화"
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

echo "==> [3/5] ROS 2 apt 저장소 등록"
ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    grep -F '"tag_name"' | awk -F'"' '{print $4}'
)
CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
echo "    ros-apt-source ${ROS_APT_SOURCE_VERSION} / ${CODENAME}"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

echo "==> [4/5] ROS 2 Humble + Nav2 + RTAB-Map 설치"
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-tools \
  ros-humble-rmw-fastrtps-cpp \
  python3-colcon-common-extensions

echo "==> [5/5] 검증"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
echo "    ROS_DISTRO=${ROS_DISTRO}"
ros2 pkg list | grep -E "^(nav2_bringup|rtabmap_slam|rtabmap_odom)$" || {
  echo "    !! 일부 패키지 누락 — 위 apt 로그 확인 필요"; exit 1;
}
echo "설치 완료. 다음: bash scripts/check_dds_bridge.sh"
