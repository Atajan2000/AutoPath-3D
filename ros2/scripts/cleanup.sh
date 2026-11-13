#!/usr/bin/env bash
# Remove leftovers of an interrupted AutoPath-3D Webots run (WSL side only; touches nothing else).
#  - webots_ros2 driver processes of this package orphaned when launch SIGTERMs webots-controller;
#    left alive they stay on the DDS graph and stall reliable topics of the next run
#  - stale Fast DDS shared-memory segments (only when no ROS 2 process is running)
P='[w]ebots_ros2_driver/driver.*autopath3d_ros'
pkill -f "$P" 2>/dev/null; sleep 1; pkill -9 -f "$P" 2>/dev/null
pkill -f "$HOME/ros2_ws/install/autopath3d_ros/lib/autopath3d_ros/" 2>/dev/null
sleep 1
if ! pgrep -f "/opt/ros/jazzy|ros2_ws/install" >/dev/null; then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
fi
pgrep -fa "$P" && echo "WARNING: drivers still running" || echo "cleanup: ok"
