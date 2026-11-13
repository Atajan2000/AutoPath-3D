# AutoPath-3D Phase 4 environment (WSL Ubuntu-24.04). Usage:  source ros2/env.sh
#
# AP3D_ROOT   the WSL checkout (a clone of the authoritative Windows repo; update with `git pull`).
# Webots runs on Windows (R2025a); the ROS side, including the extern Webots controllers, runs in WSL.
AP3D_ROOT="${AP3D_ROOT:-$HOME/autopath3d}"
AP3D_WS="${AP3D_WS:-$HOME/ros2_ws}"
export AP3D_ROOT AP3D_WS

source /opt/ros/jazzy/setup.bash
if [ -f "$AP3D_WS/install/setup.bash" ]; then
  source "$AP3D_WS/install/setup.bash"
fi

# Unchanged Phase 1-3 core library (not packaged into the ROS workspace).
export PYTHONPATH="$AP3D_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

# Windows Webots installation used through WSL interop (no second Webots). Default: the standard
# Webots R2025a install location; set WEBOTS_HOME before sourcing for a different location.
export WEBOTS_HOME="${WEBOTS_HOME:-/mnt/c/Program Files/Webots}"

# Mirrored WSL networking: /etc/resolv.conf points at 10.255.255.254 (DNS tunnel), not the Windows
# host, so extern controllers must connect to 127.0.0.1.
export AUTOPATH_WEBOTS_HOST="${AUTOPATH_WEBOTS_HOST:-127.0.0.1}"

# Measured on this machine: Fast DDS UDP between two WSL processes delivers nothing (even with
# LOCALHOST discovery / static peers) while the shared-memory transport works. All ROS processes
# run inside WSL, so use SHM only. No firewall changes are made, and with SHM as the only transport
# no DDS traffic leaves the machine. Measured: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST combined with
# SHM-only discovers nothing, the default SUBNET range works.
export FASTDDS_BUILTIN_TRANSPORTS=SHM
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
