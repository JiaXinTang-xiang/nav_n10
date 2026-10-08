# Wheeltec ROS2 Project

差分驱动小车 ROS2 Cartographer 建图、纯定位和 Nav2 导航工程。

## 目录

```text
├── ros2_ws/src/           # ROS2 工作空间
│   ├── anorosdt2/         # 飞控 IMU 桥接（Python）
│   ├── lslidar_driver/    # LSN10 激光雷达驱动（C++）
│   ├── lslidar_msgs/      # 镭神自定义消息
│   ├── wheeltec_chassis/  # 底盘串口桥接（Python 回退）
│   ├── wheeltec_chassis_cpp/ # 底盘串口桥接（生产 C++）
│   └── wheeltec_nav2/     # Nav2 导航配置和 IMU 转弯插件
├── firmware_fc/           # ANO 飞控固件（STM32F407）
├── firmware_chassis/      # 底盘固件
└── docs/                  # 交接、验收和故障排查文档
```

## 当前架构

- Jetson 运行雷达、飞控 IMU、底盘桥接、Cartographer 和 Nav2。
- PC 运行 RViz，通过 CycloneDDS 观察 Jetson 数据。
- `map -> odom` 由 Cartographer 发布。
- `odom -> base_link` 由唯一的 `chassis_bridge_cpp` 发布；原 `wheeltec_chassis/chassis_bridge` 保留作回退。
- `base_link -> laser/imu_link` 由 `robot_state_publisher` 发布。
- 纯定位模式禁止同时运行 AMCL。

## 快速开始

当前先按 [IMU 左右转验收](docs/IMU转弯验收-2026-10-08.md) 测试停车误差，通过后再执行下面的导航启动步骤。

```bash
# Jetson 编译
cd ~/Desktop/nav_n10/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Jetson 建图
cd ~/Desktop/nav_n10
bash ./slam_bringup.sh

# PC 查看建图
cd ~/桌面/nav_n10
./pc_rviz.sh slam

# 保存 Cartographer 地图
cd ~/Desktop/nav_n10
./save_cartographer_map.sh ~/Desktop/nav_n10/maps final_map

# Jetson 启动 Cartographer 纯定位
./cartographer_localization_bringup.sh \
  ~/Desktop/nav_n10/maps/final_map.pbstream

# PC 查看纯定位和导航
cd ~/桌面/nav_n10
./pc_rviz.sh nav

# Jetson 启动 Nav2，不启动 AMCL 和 map_server
cd ~/Desktop/nav_n10
./cartographer_nav2_bringup.sh
```

完整的故障排查、时间戳检查和参考工程对比见：
`docs/AI交接-建图导航与时间戳排查-2026-10-07.md`。

## 硬件

- LSN10 激光雷达（约 10 Hz）
- ANO 匿名飞控（STM32F407，ROS2 桥接后 `/imu/data` 约 100 Hz）
- 底盘 STM32 编码器差分驱动
