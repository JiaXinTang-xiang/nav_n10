#!/usr/bin/env bash
# robot_bringup.sh — 底盘 + 飞控 IMU + 雷达 一键启动
#
# 用法:  bash robot_bringup.sh    (或先 chmod +x 再 ./robot_bringup.sh)
# 串口固定名见 /etc/udev/rules.d/99-robot-serial.rules (chassis / imu / lidar)

WS="${HOME}/Desktop/nav_n10"

# 多机通信：显式声明，避免交互/非交互 shell 差异导致发现失败
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "=== [1/4] 杀掉旧进程 ==="
pkill -9 -f chassis_bridge       2>/dev/null || true
pkill -9 -f anoros_dt            2>/dev/null || true
pkill -9 -f lslidar_driver_node  2>/dev/null || true
sleep 1

echo "=== [2/4] 检查串口固定名 ==="
for p in /dev/chassis /dev/imu /dev/lidar; do
  if [ -e "$p" ]; then
    echo "  OK  $p -> $(readlink "$p")"
  else
    echo "  缺失 $p  (检查 USB 是否插上 / udev 规则是否生效)"
  fi
done

echo "=== [3/4] source 环境并启动三个节点 ==="
source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"

# nohup + 日志落盘，SSH 断开也不被杀
nohup ros2 run wheeltec_chassis chassis_bridge --ros-args \
  --params-file "${WS}/ros2_ws/src/wheeltec_chassis/config/chassis.yaml" > /tmp/chassis.log 2>&1 &
nohup ros2 run anorosdt2 anoros_dt --ros-args \
  --params-file "${WS}/ros2_ws/src/anorosdt2/config/anorosdt2.yaml" > /tmp/anoro.log 2>&1 &
nohup ros2 run lslidar_driver lslidar_driver_node --ros-args \
  --params-file "${WS}/ros2_ws/src/lslidar_driver/params/lidar_uart_ros2/lsn10.yaml" > /tmp/lidar.log 2>&1 &

echo "=== [4/4] 等待并检查话题状态 ==="
sleep 5
echo "--- 话题列表 ---"
timeout 15 ros2 topic list

echo "--- 数据流检查 (各取一帧) ---"
timeout 6 ros2 topic echo /odom     --once >/dev/null 2>&1 && echo "  /odom     OK" || echo "  /odom     无数据"
timeout 6 ros2 topic echo /imu/data --once >/dev/null 2>&1 && echo "  /imu/data OK" || echo "  /imu/data 无数据"
timeout 6 ros2 topic echo /scan     --once >/dev/null 2>&1 && echo "  /scan     OK" || echo "  /scan     无数据"

echo ""
echo "✅ 三个数据节点已启动（日志: /tmp/chassis.log /tmp/anoro.log /tmp/lidar.log）。跑 SLAM 看 RUNBOOK.md 第 4 节。"
