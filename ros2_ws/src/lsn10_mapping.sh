#!/bin/bash
# LSN10 2D激光 + 飞控IMU建图完整脚本

echo "=== 第一步：清理环境 ==="
killall -9 lslidar_driver_node cartographer_node cartographer_occupancy_grid_node robot_state_publisher rviz2 anoros_dt 2>/dev/null || true
ros2 daemon stop
ros2 daemon start
sleep 2

echo "=== 第二步：进入工作空间并设置环境 ==="
cd ~/wheeltec_project/ros2_ws
source install/setup.bash
sleep 1

echo "=== 第三步：启动雷达驱动 ==="
ros2 launch lslidar_driver lsn10_launch.py &
sleep 3

echo "=== 第四步：启动机器人状态发布器 ==="
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=false -p robot_description:='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"><visual><geometry><box size="0.5 0.5 0.2"/></geometry></visual></link><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/></joint></robot>' &
sleep 2

echo "=== 第五步：启动飞控IMU桥接 ==="
ros2 run anorosdt2 anoros_dt --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=921600 &
sleep 2

echo "=== 第六步：准备Cartographer配置 ==="
CONFIG_DIR=~/wheeltec_project/ros2_ws/install/lslidar_driver/share/lslidar_driver/config
cp ${CONFIG_DIR}/lsn10.lua /tmp/lsn10_complete.lua
sleep 1

echo "=== 第七步：启动Cartographer建图 ==="
ros2 run cartographer_ros cartographer_node -configuration_directory /tmp -configuration_basename lsn10_complete.lua --ros-args -p use_sim_time:=false -p tracking_frame:=imu_link -p published_frame:=odom -p num_laser_scans:=1 -p num_multi_echo_laser_scans:=0 -p imu_sampling_ratio:=1 --remap scan:=scan --remap imu:=/imu/data &
sleep 3

echo "=== 第八步：启动地图网格 ==="
ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p use_sim_time:=false -p resolution:=0.05 &
sleep 2

echo "=== 第九步：启动RViz ==="
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=false &
sleep 3

echo "=== 第十步：检查系统状态 ==="
echo "节点列表："
ros2 node list
echo ""
echo "相关话题："
ros2 topic list | grep -E "(scan|map|tf|imu)"
echo ""
echo "IMU数据检查："
ros2 topic echo /imu/data --once 2>/dev/null || echo "  (IMU 尚未收到数据，请检查飞控串口连接)"
echo ""
echo "雷达数据检查："
ros2 topic echo /scan --once
echo ""
echo "TF树检查："
ros2 run tf2_tools view_frames

echo ""
echo "=== 建图系统启动完成！==="
echo "请在RViz中进行以下设置："
echo "1. Fixed Frame 设置为 'map'"
echo "2. 添加 LaserScan 显示（话题：/scan）"
echo "3. 添加 Map 显示（话题：/map）"
echo "4. 开始缓慢移动雷达进行建图"
echo ""
echo "建图完成后，使用以下命令保存地图："
echo "ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename ~/.ros/cartographer.pbstream -map_filestem ./my_map -resolution 0.05"
