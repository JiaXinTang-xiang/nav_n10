#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=/data/logs/robot_navigation

for attempt in $(seq 1 40); do
  if [[ -c /dev/ttyUSB0 ]]; then
    exec ros2 launch robot_slam localization_runtime.launch.py
  fi
  sleep 0.25
done

echo "RPLIDAR device /dev/ttyUSB0 did not appear within 10 seconds" >&2
exit 1
