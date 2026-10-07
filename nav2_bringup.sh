#!/usr/bin/env bash
# Jetson 导航一键启动：清理旧进程后启动雷达、IMU、底盘、AMCL 和 Nav2。

set -eo pipefail

WS="${HOME}/Desktop/nav_n10"
MAP="${1:-${WS}/maps/map_20261006_231058.yaml}"

if [ ! -f "${WS}/ros2_ws/install/setup.bash" ]; then
  echo "❌ ROS2 工作空间未编译: ${WS}/ros2_ws/install/setup.bash"
  exit 1
fi

if [ ! -f "${MAP}" ]; then
  echo "❌ 地图不存在: ${MAP}"
  exit 1
fi

for device in /dev/lidar /dev/imu /dev/chassis; do
  if [ ! -e "${device}" ]; then
    echo "❌ 缺少设备 ${device}"
    exit 1
  fi
done

source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"

echo "=== 清理旧 SLAM、导航和串口节点 ==="
pkill -TERM -f "ros2 launch wheeltec_nav2 navigation.launch.py" 2>/dev/null || true
pkill -TERM -f "ros2 launch.*cartographer" 2>/dev/null || true
sleep 2

for pattern in \
  cartographer_node \
  cartographer_occupancy_grid_node \
  lslidar_driver_node \
  anoros_dt \
  chassis_bridge \
  robot_state_publisher \
  map_server \
  amcl \
  controller_server \
  planner_server \
  behavior_server \
  bt_navigator \
  velocity_smoother \
  navigation_autostarter \
  lifecycle_manager; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done

sleep 1

for pattern in \
  cartographer_node \
  cartographer_occupancy_grid_node \
  lslidar_driver_node \
  anoros_dt \
  chassis_bridge \
  robot_state_publisher \
  map_server \
  amcl \
  controller_server \
  planner_server \
  behavior_server \
  bt_navigator \
  velocity_smoother \
  navigation_autostarter \
  lifecycle_manager; do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done

for device in /dev/lidar /dev/imu /dev/chassis; do
  fuser -k "${device}" >/dev/null 2>&1 || true
done

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "=== 串口 ==="
for device in /dev/lidar /dev/imu /dev/chassis; do
  echo "${device} -> $(readlink -f "${device}")"
done

echo "=== 启动导航 ==="
echo "地图: ${MAP}"
echo "启动后在 PC RViz 使用 2D Pose Estimate 设置初始位置。"
echo "AMCL 收到初始位姿后，脚本会自动激活其余 Nav2 节点。"

exec ros2 launch wheeltec_nav2 navigation.launch.py \
  map:="${MAP}" \
  rviz:=false
