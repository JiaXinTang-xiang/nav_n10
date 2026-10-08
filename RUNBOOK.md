# RUNBOOK — 底盘 + SLAM 运行手册 / 命令速查

> 本文档只记「怎么跑起来」的命令，速查用。
> 架构和踩坑看 [HANDOFF.md](HANDOFF.md)，硬件细节看 [docs/底盘硬件资源表与编码器标定.md](docs/底盘硬件资源表与编码器标定.md)。

---

## 0. 串口固定名（已用 udev 规则固定，不用再查编号）

规则文件：`/etc/udev/rules.d/99-robot-serial.rules`（源文件在 [udev/99-robot-serial.rules](udev/99-robot-serial.rules)）。

| 设备 | 固定名 | 波特率 | 节点 | 发布话题 |
|---|---|---|---|---|
| 底盘 F407 | `/dev/chassis` | 115200 | `chassis_bridge` | `/odom`、`/cmd_vel` |
| 飞控 IMU | `/dev/imu` | 921600 | `anoros_dt` | `/imu/data` |
| LSN10 雷达 | `/dev/lidar` | 230400 | `lslidar_driver_node` | `/scan` |

> ⚠️ 底盘和飞控靠「物理 USB 口」区分（CH340 无序列号）。**这两根线别换 USB 口**；换了就重跑：
> `udevadm info -a -n /dev/ttyCH341USB* | grep KERNELS` 更新规则里的 `1-2.x` 路径。

---

## 1. 每次开机（新终端）都要先做的

```bash
# 1) source 环境（每个新终端都要）
source /opt/ros/humble/setup.bash
source ~/Desktop/nav_n10/ros2_ws/install/setup.bash

# 2) 确认三个固定串口都在
ls -l /dev/chassis /dev/imu /dev/lidar
```

预期：三个软链接都指向真实设备（`chassis→ttyCH341USB*`、`imu→ttyCH341USB*`、`lidar→ttyACM0`）。

---

## 2. 单独测试每个传感器

### 2.1 激光雷达（LSN10）

```bash
ros2 run lslidar_driver lslidar_driver_node --ros-args \
  -p lidar_name:=N10 -p interface_selection:=serial \
  -p serial_port_:=/dev/lidar -p frame_id:=laser

# 另开终端验证（应 ~10Hz）
ros2 topic hz /scan
```

驱动日志出现 `open_port /dev/lidar OK !` 即串口打开成功。

### 2.2 底盘（里程计）

```bash
ros2 run wheeltec_chassis_cpp chassis_bridge_cpp --ros-args \
  --params-file ~/Desktop/nav_n10/ros2_ws/src/wheeltec_chassis_cpp/config/chassis.yaml

# 验证（应 ~50Hz）
ros2 topic hz /odom
ros2 topic echo /odom --once      # 推一下车，pose.position.x/y 应变化
```

### 2.3 飞控 IMU

```bash
ros2 run anorosdt2 anoros_dt --ros-args -p serial_port:=/dev/imu

# 验证（应 ~1kHz）
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once  # angular_velocity / orientation 应非零
```

---

## 3. 一键启动全部节点（bringup）

**最快方式**：直接跑脚本（自动杀旧进程 → 起三个节点 → 打印状态）：

```bash
bash ~/Desktop/nav_n10/robot_bringup.sh
```

手动方式（等价于脚本做的事）：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/nav_n10/ros2_ws/install/setup.bash

# 杀掉可能残留的旧进程
pkill -9 -f chassis_bridge
pkill -9 -f anoros_dt
pkill -9 -f lslidar_driver_node

# 起三个数据节点
ros2 run wheeltec_chassis_cpp chassis_bridge_cpp --ros-args \
  --params-file ~/Desktop/nav_n10/ros2_ws/src/wheeltec_chassis_cpp/config/chassis.yaml &
ros2 run anorosdt2 anoros_dt --ros-args -p serial_port:=/dev/imu &
ros2 run lslidar_driver lslidar_driver_node --ros-args \
  -p lidar_name:=N10 -p interface_selection:=serial \
  -p serial_port_:=/dev/lidar -p frame_id:=laser &

# 三个都该有数据
ros2 topic hz /odom /imu/data /scan
```

---

## 4. 跑 SLAM 建图（Cartographer）

> 前置：第 3 步的三个数据节点已经在跑（底盘 odom + 飞控 IMU + 雷达 scan）。

### 4.1 起机器人状态发布器（TF: base_link → laser / imu_link）

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"/><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>'
```

### 4.2 起 Cartographer 建图节点

```bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory ~/Desktop/nav_n10/ros2_ws/install/lslidar_driver/share/lslidar_driver/config \
  -configuration_basename lsn10.lua \
  --ros-args -p use_sim_time:=false --remap imu:=/imu/data
```

### 4.3 起占据栅格地图 + RViz

```bash
ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 &

ros2 run rviz2 rviz2 -d ~/Desktop/nav_n10/ros2_ws/src/lslidar_driver/rviz/lsn10_cartographer.rviz
```

### 4.4 建图前先验证 TF 树（阶段 3 第一件事）

```bash
ros2 run tf2_tools view_frames   # 生成 frames.pdf，看 TF 树有没有断/冲突
```

⚠️ **已知风险**：`lsn10.lua` 里 `provide_odom_frame=true`，Cartographer 和 `chassis_bridge` 都可能发 `odom→base_link`，会 TF 冲突。若 `view_frames` 报冲突，先改 `lsn10.lua` 的 `provide_odom_frame=false` 再试（改完重新 `colcon build`）。

### 4.5 开始建图

1. RViz 里 `Fixed Frame` 设成 `map`，加 `LaserScan`(/scan) 和 `Map`(/map) 显示。
2. 用键盘慢慢推车：`ros2 run teleop_twist_keyboard teleop_twist_keyboard`（确认前进=前进、左转=左转）。
3. 慢速走遍要建图的区域，绕一圈回起点验证回环。

---

## 5. 保存地图（建图完成后）

```bash
# 1) 触发写状态，生成 .pbstream
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${HOME}/map.pbstream', include_unfinished_submaps: true}"

# 2) 转成 Nav2 能用的 occupancy grid 地图
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename ${HOME}/map.pbstream -map_filestem ${HOME}/map -resolution 0.05
```

> 服务名若对不上，`ros2 service list | grep write` 查一下实际名字。

---

## 6. 导航（Nav2，阶段 4，待做）

```bash
# 需要先有第 5 步保存的地图
ros2 launch wheeltec_nav2 navigation.launch.py map:=${HOME}/map.yaml
```

---

## 7. 常见问题排查

| 现象 | 排查 |
|---|---|
| `/odom` 或 `/imu/data` 没数据 | ① `ls -l /dev/chassis /dev/imu` 软链接在不在 ② 旧节点还占着失效串口 → `pkill -9 -f chassis_bridge` / `-f anoros_dt` 后重启 |
| `/scan` 没数据 | ① `/dev/lidar` 软链接 ② 驱动日志有没有 `open_port ... OK !` ③ `ros2 topic hz /scan` |
| 串口号漂移（拔插后） | 已用 udev 固定，看第 0 节；若换了物理 USB 口需更新规则 |
| 建图地图飘/重影 | TF 冲突（4.4 节）、IMU 安装方向、轮距 `WHEEL_SEPARATION_M` 未实测 |
| `ros2` 命令找不到 | 先 `source /opt/ros/humble/setup.bash` |

---

## 附：当前待办（收尾项）

- [x] 三个 yaml 的 `serial_port` 已改成固定名（`chassis.yaml`→`/dev/chassis`、`anorosdt2.yaml`→`/dev/imu`、`lsn10.yaml`→`/dev/lidar`），已 `colcon build` 生效。
- [x] `robot_bringup.sh` 一键启动脚本已写好：`bash ~/Desktop/nav_n10/robot_bringup.sh`。
- [ ] 阶段 3 的 TF 冲突 / 轮距标定验证。



终端 1 — 起三路数据

bash ~/Desktop/nav_n10/robot_bringup.sh
确认输出末尾三行都是 OK（/odom、/imu/data、/scan）。

终端 2 — 起 SLAM 栈（RSP + Cartographer）

source /opt/ros/humble/setup.bash
source ~/Desktop/nav_n10/ros2_ws/install/setup.bash

# 机器人状态发布器 (TF: base_link → laser / imu_link)
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:='<?xml version="1.0"?><robot name="lsn10_robot"><link name="base_link"/><link name="imu_link"/><link name="laser"/><joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint><joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0 0 0.1" rpy="0 0 0"/></joint></robot>' &

# Cartographer 建图节点（前台跑，看日志）
ros2 run cartographer_ros cartographer_node \
  -configuration_directory ~/Desktop/nav_n10/ros2_ws/install/lslidar_driver/share/lslidar_driver/config \
  -configuration_basename lsn10.lua \
  --ros-args -p use_sim_time:=false --remap imu:=/imu/data
终端 3 — 地图 + RViz

source /opt/ros/humble/setup.bash
source ~/Desktop/nav_n10/ros2_ws/install/setup.bash

ros2 run cartographer_ros cartographer_occupancy_grid_node --ros-args -p resolution:=0.05 &
ros2 run rviz2 rviz2 -d ~/Desktop/nav_n10/ros2_ws/src/lslidar_driver/rviz/lsn10_cartographer.rviz
终端 4 — 查 TF 树（这是阶段 3 的第一道关）

source /opt/ros/humble/setup.bash
source ~/Desktop/nav_n10/ros2_ws/install/setup.bash

ros2 run tf2_tools view_frames
看生成的 frames.pdf，重点检查有没有 odom→base_link 被两个节点同时发布（Cartographer 和 chassis_bridge）。完整链路应该是：


map → base_link → laser
              └→ imu_link
如果 TF 冲突（两个 odom→base_link）
改 lsn10.lua 里这一行，然后重新 colcon build：


# 把
provide_odom_frame = true
# 改成
provide_odom_frame = false

cd ~/Desktop/nav_n10/ros2_ws && source /opt/ros/humble/setup.bash && colcon build
改完重跑终端 2 的 Cartographer。

建图操作
RViz 里 Fixed Frame 设 map，加 LaserScan(/scan) 和 Map(/map)。
键盘推车，先确认方向对（前进=前进、左转=左转）：

ros2 run teleop_twist_keyboard teleop_twist_keyboard
慢速走遍要建图的区域，最后绕回起点验证回环。
建图完成后保存地图

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${HOME}/map.pbstream', include_unfinished_submaps: true}"

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename ${HOME}/map.pbstream -map_filestem ${HOME}/map -resolution 0.05
