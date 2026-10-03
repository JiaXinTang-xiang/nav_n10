#!/usr/bin/env bash
# slam_bringup.sh — 一键启动：底盘+IMU+雷达 + RSP + Cartographer + 地图
# 用法: bash ~/Desktop/nav_n10/slam_bringup.sh

WS="${HOME}/Desktop/nav_n10"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"
source /opt/ros/humble/setup.bash

echo "=== [1/5] 清理旧进程(含导航节点，避免和 Cartographer 抢 map→odom) ==="
pkill -9 -f chassis_bridge                2>/dev/null
pkill -9 -f anoros_dt                     2>/dev/null
pkill -9 -f lslidar_driver_node           2>/dev/null
pkill -9 -f robot_state_publisher         2>/dev/null
pkill -9 -f cartographer_node             2>/dev/null
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null
pkill -9 -f amcl                          2>/dev/null
pkill -9 -f map_server                    2>/dev/null
pkill -9 -f controller_server             2>/dev/null
pkill -9 -f planner_server                2>/dev/null
pkill -9 -f behavior_server               2>/dev/null
pkill -9 -f bt_navigator                  2>/dev/null
pkill -9 -f lifecycle_manager             2>/dev/null
pkill -9 -f velocity_smoother             2>/dev/null
sleep 2
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "=== [2/5] 三路数据 ==="
source "${WS}/ros2_ws/install/setup.bash"
nohup ros2 run wheeltec_chassis chassis_bridge --ros-args \
  --params-file "${WS}/ros2_ws/src/wheeltec_chassis/config/chassis.yaml" \
  -p publish_tf:=false > /tmp/chassis.log 2>&1 &
nohup ros2 run anorosdt2 anoros_dt --ros-args \
  --params-file "${WS}/ros2_ws/src/anorosdt2/config/anorosdt2.yaml" > /tmp/anoro.log 2>&1 &
nohup ros2 run lslidar_driver lslidar_driver_node --ros-args \
  --params-file "${WS}/ros2_ws/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml" > /tmp/lidar.log 2>&1 &
sleep 3

echo "=== [3/5] robot_state_publisher (TF: base_link→laser/imu) ==="
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"/><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>' > /tmp/rsp.log 2>&1 &
sleep 2

echo "=== [4/5] Cartographer 建图 ==="
nohup ros2 run cartographer_ros cartographer_node \
  -configuration_directory "${WS}/ros2_ws/install/lslidar_driver/share/lslidar_driver/config" \
  -configuration_basename lsn10.lua \
  --ros-args -p use_sim_time:=false \
  --remap imu:=/imu/data --remap odom:=/odom > /tmp/carto.log 2>&1 &
sleep 3

echo "=== [5/5] 占据栅格地图 ==="
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 > /tmp/occgrid.log 2>&1 &
sleep 4

echo
echo "=== 节点 ==="
ros2 node list 2>&1
echo "=== 话题 ==="
ros2 topic list 2>&1 | grep -E "map|scan|odom|imu|tf"
echo
if pgrep -f cartographer_node >/dev/null; then
  echo "✅ SLAM 栈已启动，PC 上开 RViz 看 /scan 和 /map"
else
  echo "❌ cartographer_node 没起来，看 /tmp/carto.log"
fi
