#!/bin/bash
# AutoPath-3D Phase 4: install ROS 2 Jazzy (ros-base) + webots_ros2 driver/crazyflie on Ubuntu 24.04 (WSL).
#
# Run it yourself in an Ubuntu 24.04 (WSL) terminal; sudo will prompt for your password:
#   bash ros2/setup/install_ros2_jazzy.sh
# Optional: AP3D_PROXY=http://host:port routes only these apt/curl calls through an HTTP proxy.
#
# Follows https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html (ros2-apt-source package,
# no apt-key). Only the packages AutoPath-3D needs are installed (not the webots_ros2 meta-package, which
# pulls TIAGo/UR/Husarion/Nav2 stacks). If AP3D_PROXY is set, apt uses that proxy only for these commands;
# no system-wide apt or network configuration is changed.
set -euo pipefail

PROXY="${AP3D_PROXY:-}"                 # empty: direct connection (default)
APT=(sudo apt-get)
if [ -n "$PROXY" ]; then
  APT+=(-o "Acquire::http::Proxy=${PROXY}" -o "Acquire::https::Proxy=${PROXY}")
  export https_proxy="$PROXY"
fi
BUDGET_MB="${AP3D_BUDGET_MB:-6000}"     # abort if the install would need more than this
RESERVE_MB=5000                         # keep at least this much free afterwards
PKGS=(
  ros-jazzy-ros-base            # rclpy, rclcpp, launch, launch_ros, tf2_ros, common msgs
  ros-dev-tools                 # colcon, rosdep, vcstool, ament tooling
  ros-jazzy-webots-ros2-driver  # Webots <-> ROS 2 driver (2025.0.0, matches Webots R2025a)
  ros-jazzy-webots-ros2-crazyflie   # Bitcraze Crazyflie PID flight controller plugin
  python3-numpy python3-scipy python3-pytest
)
CACHE="$HOME/.cache/ap3d"
mkdir -p "$CACHE"

echo "== Free space before: $(df -m --output=avail / | tail -1) MB"

if [ ! -f /etc/apt/sources.list.d/ros2.sources ]; then
  V=$(curl -s --max-time 30 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
  DEB="$CACHE/ros2-apt-source_${V}.${CODENAME}_all.deb"
  [ -f "$DEB" ] || curl -sL --max-time 60 -o "$DEB" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${CODENAME}_all.deb"
  dpkg-deb -I "$DEB" | grep -q 'Package: ros2-apt-source'
  sudo dpkg -i "$DEB"
fi

"${APT[@]}" update

# Size preflight (dry run)
NPKG=$("${APT[@]}" install -s "${PKGS[@]}" 2>/dev/null | awk '/^Inst /{n++} END{print n+0}')
SUMMARY=$("${APT[@]}" install --assume-no "${PKGS[@]}" 2>/dev/null | grep -E 'additional disk space|newly installed' || true)
echo "== Dry run: ${NPKG} packages to install"
echo "$SUMMARY"
NEED_MB=$(echo "$SUMMARY" | sed -n 's/.*After this operation, \([0-9.,]*\) \([kMG]B\).*/\1 \2/p' | \
          awk '{gsub(",","",$1); m=$1; if($2=="kB")m/=1024; if($2=="GB")m*=1024; printf "%d", m}')
FREE_MB=$(df -m --output=avail / | tail -1)
HOST_FREE_MB=$(df -m --output=avail /mnt/c | tail -1)   # the ext4.vhdx grows on C:, the real limit
[ "$HOST_FREE_MB" -lt "$FREE_MB" ] && FREE_MB=$HOST_FREE_MB
echo "== Estimated: ${NEED_MB:-?} MB; free: ${FREE_MB} MB; budget: ${BUDGET_MB} MB; reserve: ${RESERVE_MB} MB"
if [ -n "${NEED_MB}" ] && { [ "$NEED_MB" -gt "$BUDGET_MB" ] || [ $((FREE_MB - NEED_MB)) -lt "$RESERVE_MB" ]; }; then
  echo "ABORT: install exceeds budget or reserve. Nothing installed beyond the apt source." >&2
  exit 2
fi

"${APT[@]}" install -y "${PKGS[@]}"

echo "== Free space after: $(df -m --output=avail / | tail -1) MB"
source /opt/ros/jazzy/setup.bash
echo "== ROS_DISTRO=$ROS_DISTRO"
dpkg -l | awk '/ros-jazzy-webots-ros2/{print $2, $3}'
echo "INSTALL_OK"
