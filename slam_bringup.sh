#!/usr/bin/env bash
# slam_bringup.sh — 一键启动：底盘+IMU+雷达 + RSP + Cartographer + 地图
# 用法: bash ~/Desktop/nav_n10/slam_bringup.sh

# 始终使用本脚本所在的工程根目录，避免沿用旧工程 ~/Desktop/nav_n10。
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"
source /opt/ros/humble/setup.bash

if [ ! -f "${WS}/ros2_ws/install/setup.bash" ]; then
  echo "❌ 找不到 ${WS}/ros2_ws/install/setup.bash"
  echo "请先在工程的 ros2_ws 目录执行 colcon build，再重新运行本脚本。"
  exit 1
fi

for device in /dev/imu /dev/lidar; do
  if [ ! -e "${device}" ]; then
    echo "❌ 缺少设备 ${device}，停止启动SLAM。"
    exit 1
  fi
done

CHASSIS_DEVICE=/dev/chassis
if [ ! -e "${CHASSIS_DEVICE}" ]; then
  imu_device="$(readlink -f /dev/imu 2>/dev/null || true)"
  chassis_candidates=()
  shopt -s nullglob
  for candidate in /dev/ttyCH341USB* /dev/ttyUSB*; do
    if [ "$(readlink -f "${candidate}")" != "${imu_device}" ]; then
      chassis_candidates+=("${candidate}")
    fi
  done
  shopt -u nullglob

  if [ "${#chassis_candidates[@]}" -eq 1 ]; then
    CHASSIS_DEVICE="${chassis_candidates[0]}"
    echo "⚠️ /dev/chassis 不存在，自动使用底盘串口 ${CHASSIS_DEVICE}"
  else
    echo "❌ 找不到独立的底盘串口，停止启动SLAM。"
    if command -v lsusb >/dev/null 2>&1 && \
       lsusb -v -d 1a86:7523 2>/dev/null | grep -q "USB MIDI"; then
      echo "❌ 检测到底盘USB被枚举成 USB MIDI，而不是 USB Serial。"
      echo "   此设备没有 tty 串口接口，不能把 /dev/imu 当作底盘。"
    fi
    echo "当前串口设备："
    ls -l /dev/ttyCH341USB* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    exit 1
  fi
fi

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
pkill -9 -f start_cartographer_localization 2>/dev/null
sleep 2
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "=== [2/5] 三路数据 ==="
source "${WS}/ros2_ws/install/setup.bash"
nohup ros2 run wheeltec_chassis chassis_bridge --ros-args \
  --params-file "${WS}/ros2_ws/src/wheeltec_chassis/config/chassis.yaml" \
  -p serial_port:="${CHASSIS_DEVICE}" \
  -p publish_tf:=true > /tmp/chassis.log 2>&1 &
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
nohup ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args \
  -p resolution:=0.05 -p publish_period_sec:=5.0 > /tmp/occgrid.log 2>&1 &
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
