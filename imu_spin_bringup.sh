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

if ! nodes="$(timeout 8s ros2 node list --no-daemon --spin-time 2.0)"; then
  echo "无法检查控制节点，停止启动；请检查 ROS/DDS 环境。"
  exit 1
fi
if grep -Eq '^/(controller_server|behavior_server|imu_spin_controller)$' <<< "${nodes}"; then
  echo "请先关闭 Nav2 和已有转弯节点，禁止多个控制器同时控制底盘。"
  exit 1
fi
echo "测试前关闭键盘控制与其他速度发布器；独立 /imu_spin 不提供避障。"

exec ros2 run wheeltec_nav2 imu_spin_controller.py --ros-args \
  -p min_angular_speed:=0.55 \
  -p max_angular_speed:=0.60 \
  -p kp:=1.8 \
  -p slow_down_angle:=0.45 \
  -p angle_tolerance:=0.045
