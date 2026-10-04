#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=/data/logs/robot_navigation
mkdir -p /data/projects/robot_navigation "$ROS_LOG_DIR"
# Sensors, wheel odometry and localization already belong to their existing services.
# Wait for localization before Nav2 lifecycle activation, without inventing a pose.
ros2 run robot_navigation wait_for_localization.py
exec ros2 launch robot_navigation navigation_v1.launch.py \
  start_localization_runtime:=false \
  latch_file:=/data/projects/robot_navigation/stop_latch
