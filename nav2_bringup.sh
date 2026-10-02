#!/usr/bin/env bash
# nav2_bringup.sh — 导航栈一键启动（AMCL 定位 + Nav2）
# 用法: bash ~/Desktop/nav_n10/nav2_bringup.sh [地图yaml路径，默认 ~/map.yaml]

WS="${HOME}/Desktop/nav_n10"
MAP="${1:-${HOME}/map.yaml}"
PARAMS="${WS}/ros2_ws/src/wheeltec_nav2/config/nav2_params.yaml"
URDF='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"/><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>'

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"

echo "=== [1/4] 清理旧进程(含 Cartographer，导航改用 AMCL) ==="
pkill -9 -f chassis_bridge 2>/dev/null
pkill -9 -f anoros_dt 2>/dev/null
pkill -9 -f lslidar_driver_node 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f cartographer_node 2>/dev/null
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null
pkill -9 -f map_server 2>/dev/null
pkill -9 -f amcl 2>/dev/null
pkill -9 -f controller_server 2>/dev/null
pkill -9 -f planner_server 2>/dev/null
pkill -9 -f behavior_server 2>/dev/null
pkill -9 -f bt_navigator 2>/dev/null
pkill -9 -f velocity_smoother 2>/dev/null
pkill -9 -f lifecycle_manager 2>/dev/null
pkill -9 -f "ros2 run" 2>/dev/null
pkill -9 -f ros2-daemon 2>/dev/null
sleep 2
ros2 daemon start >/dev/null 2>&1

echo "=== [2/4] 数据节点(底盘/雷达/URDF) ==="
nohup ros2 run wheeltec_chassis chassis_bridge --ros-args \
  --params-file "${WS}/ros2_ws/src/wheeltec_chassis/config/chassis.yaml" > /tmp/chassis.log 2>&1 &
nohup ros2 run lslidar_driver lslidar_driver_node --ros-args \
  --params-file "${WS}/ros2_ws/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml" > /tmp/lidar.log 2>&1 &
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="${URDF}" > /tmp/rsp.log 2>&1 &
sleep 3

echo "=== [3/4] Nav2 生命周期节点 ==="
nohup ros2 run nav2_map_server map_server --ros-args \
  --params-file "${PARAMS}" -p yaml_filename:="${MAP}" > /tmp/map_server.log 2>&1 &
nohup ros2 run nav2_amcl amcl --ros-args \
  --params-file "${PARAMS}" > /tmp/amcl.log 2>&1 &
nohup ros2 run nav2_controller controller_server --ros-args \
  --params-file "${PARAMS}" > /tmp/controller.log 2>&1 &
nohup ros2 run nav2_planner planner_server --ros-args \
  --params-file "${PARAMS}" > /tmp/planner.log 2>&1 &
nohup ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file "${PARAMS}" > /tmp/behavior.log 2>&1 &
nohup ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file "${PARAMS}" > /tmp/bt.log 2>&1 &
sleep 2

echo "=== [4/4] lifecycle_manager 配置+激活 ==="
nohup ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p "node_names:=['map_server','amcl','controller_server','planner_server','behavior_server','bt_navigator']" \
  -p autostart:=true > /tmp/lifecycle.log 2>&1 &
sleep 6

echo
echo "=== 节点 ==="
ros2 node list 2>&1
echo "=== 话题 ==="
ros2 topic list 2>&1 | grep -E "map|amcl|cmd_vel|plan|scan|odom|costmap"
echo
echo "✅ 导航栈已启动。PC 上开 RViz：先 2D Pose Estimate 给初始位姿，再 2D Nav Goal 给目标点。"
