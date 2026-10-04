#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
exec rviz2 -d "${XDG_DATA_HOME:-$HOME/.local/share}/robot_navigation/navigation.rviz" "$@"
