#!/usr/bin/env bash
# Start sensors, wheel odometry and Cartographer pure localization only.
# This script does not start Nav2 and does not publish motion commands.

set -euo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PBSTREAM="${1:-}"

if [ -z "${PBSTREAM}" ] || [ ! -s "${PBSTREAM}" ]; then
  echo "用法: $0 /绝对路径/map.pbstream"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"

for device in /dev/lidar /dev/imu /dev/chassis; do
  if [ ! -e "${device}" ]; then
    echo "❌ 缺少设备 ${device}"
    exit 1
  fi
done

echo "=== 清理旧建图、定位和导航进程 ==="
for i in $(seq 1 5); do
  timeout 2s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
done
for pattern in \
  cartographer_node cartographer_occupancy_grid_node \
  lslidar_driver_node anoros_dt chassis_bridge robot_state_publisher \
  map_server amcl controller_server planner_server behavior_server \
  bt_navigator velocity_smoother navigation_autostarter lifecycle_manager; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 2

CONFIG_DIR="${WS}/ros2_ws/install/lslidar_driver/share/lslidar_driver/config"

echo "=== 启动传感器和底盘里程计 ==="
nohup ros2 run wheeltec_chassis chassis_bridge --ros-args \
  --params-file "${WS}/ros2_ws/src/wheeltec_chassis/config/chassis.yaml" \
  -p serial_port:=/dev/chassis -p publish_tf:=true \
  > /tmp/chassis.log 2>&1 &
nohup ros2 run anorosdt2 anoros_dt --ros-args \
  --params-file "${WS}/ros2_ws/src/anorosdt2/config/anorosdt2.yaml" \
  > /tmp/anoro.log 2>&1 &
nohup ros2 run lslidar_driver lslidar_driver_node --ros-args \
  --params-file "${WS}/ros2_ws/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml" \
  > /tmp/lidar.log 2>&1 &

ROBOT_DESCRIPTION='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"/><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>'
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="${ROBOT_DESCRIPTION}" > /tmp/rsp.log 2>&1 &
sleep 3

echo "=== 加载 pbstream，等待 RViz 设置初始位姿 ==="
nohup ros2 run cartographer_ros cartographer_node \
  -configuration_directory "${CONFIG_DIR}" \
  -configuration_basename lsn10_localization.lua \
  -load_state_filename "${PBSTREAM}" \
  -load_frozen_state=true \
  -start_trajectory_with_default_topics=false \
  --ros-args --remap scan:=/scan --remap odom:=/odom \
  > /tmp/carto_localization.log 2>&1 &
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args -p resolution:=0.05 -p publish_period_sec:=5.0 \
  > /tmp/occgrid_localization.log 2>&1 &
nohup ros2 run wheeltec_nav2 start_cartographer_localization.py --ros-args \
  -p configuration_directory:="${CONFIG_DIR}" \
  -p configuration_basename:=lsn10_localization.lua \
  > /tmp/start_localization.log 2>&1 &
sleep 4

echo "✅ Cartographer 纯定位已启动，Nav2未启动。"
echo "请在 PC RViz 使用 2D Pose Estimate 设置当前位置。"
echo "日志: /tmp/carto_localization.log /tmp/start_localization.log"
