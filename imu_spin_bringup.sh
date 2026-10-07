#!/usr/bin/env bash
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"

set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "${WS}/cyclonedds/cyclonedds-jetson.xml" ]; then
  export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"
elif [ -f "${WS}/cyclonedds/cyclonedds-pc.xml" ]; then
  export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-pc.xml"
fi

pkill -TERM -f 'wheeltec_nav2/lib/wheeltec_nav2/imu_spin_controller' 2>/dev/null || true
sleep 1

exec ros2 run wheeltec_nav2 imu_spin_controller.py --ros-args \
  -p min_angular_speed:=0.60 \
  -p max_angular_speed:=0.80 \
  -p kp:=1.8 \
  -p slow_down_angle:=0.45 \
  -p angle_tolerance:=0.045
