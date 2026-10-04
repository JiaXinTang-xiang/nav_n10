# RViz2 与实车导航

[English](rviz_vm.md)

**PASS — 2026-09-13：**操作者确认全图及薄墙导航成功；记录中的六个显式目标全部
`SUCCEEDED`，没有进度失败，到达后正常撤权。本次覆盖初始定位后的点到点运动。
集成遇障／安全场景和完整重启至到达流程另行验收。

## 在 Ubuntu VM 运行 RViz

Orange Pi 运行 ROS、定位和 Nav2，VM 只运行 RViz。VMware 网络使用
**Bridged（桥接）**，与 `robot-core.local` 位于同一局域网。在 VM 中仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d ./ros2_ws/robot_navigation/config/navigation.rviz
```

使用 VM 内可访问的 Linux 路径。需要安装 **Robot Navigation RViz** 启动器和登录
自启动项时，执行一次：

```bash
bash ./ros2_ws/robot_navigation/scripts/install_rviz_user.sh
```

安装器会备份已有 RViz 默认配置。桌面登录自启动已配置；手动启动器运行已验证。

## 操作 demo

1. 让机器人保持静止，在 RViz 中打开保存的地图。
2. 点击 **2D Pose Estimate**，在机器人实际位置按住鼠标，沿车头方向拖出箭头。
3. 确认紫色激光点与地图墙面重合；定位完成后 Nav2 自动激活。
4. 使用 **2D Goal Pose** 指定目标位置与朝向。**发送目标后，健康检查通过即自主出发。**
5. 机器人沿绿色路径行驶，到达后停车并撤销运动授权。

| 显示项 | 话题 |
|---|---|
| 地图 | `/map` |
| 紫色激光点 | `/scan` |
| 小车里程计 | `/odom` |
| 绿色全局路径 | `/plan` |
| 小车轮廓 | `/local_costmap/published_footprint` |

Fixed Frame 为 `map`。初始定位前，地图坐标下的雷达点和机器人位姿可能不可见。
Cartographer 发布 `map -> odom`，桥接发布 `odom -> base_link`，传感器变换节点发布
`base_link -> laser_frame`。不要重复启动定位或 TF 发布者。

在同一 ROS domain、已加载 ROS Humble 环境的终端操作：

```bash
ros2 service call /navigation_safety_gate/stop std_srvs/srv/Trigger '{}'
# 排除 STOP／故障原因后才复位；复位不会恢复旧目标：
ros2 service call /navigation_safety_gate/reset std_srvs/srv/Trigger '{}'
```

STOP／故障锁存跨重启保留。导航目标只由用户显式发送，不持久化或重放。
使用 `/goal_pose`，不要通过直接调用 Nav2 action 或桥接 arm 绕过现有运动门控。

## Orange Pi 部署

ROS 工作空间为 `/data/ros2_ws`，运行时 Python 为 `/usr/bin/python3`。
参见[桥接部署](ros_bridge.zh-CN.md)和[建图／定位](slam_mapping.zh-CN.md)。
`ros2_ws/slam_results/` 中的地图对应项目测试场地；在该场地使用时，将其中
`.pbstream`、`.pgm`、`.yaml` 复制到 `/data/projects/maps/`。
其他场地需建立对应地图，并设置机器人的实际初始位姿。

构建并安装 `robot_slam` 与 `robot_navigation` 后，在 Orange Pi 安装一次服务定义：

```bash
sudo install -m 0644 /data/ros2_ws/install/robot_slam/share/robot_slam/systemd/robot-localization.service /etc/systemd/system/
sudo install -m 0644 /data/ros2_ws/install/robot_navigation/share/robot_navigation/systemd/robot-navigation.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-localization.service robot-navigation.service
```

桥接服务保持独立。定位服务启动雷达、传感器 TF 和 Cartographer，再等待 `/initialpose`；
导航服务等待定位有效后激活 Nav2。服务激活和初始位姿交接已验证，不代表完整重启至
到达流程已验收。

仅在**导航服务未运行**、传感器和定位已启动并初始化时，可手动复用它们：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
ros2 launch robot_navigation navigation_v1.launch.py start_localization_runtime:=false
```

STOP／故障后的维护启动需追加 `startup_locked:=true` 保留锁存；恢复健康后再人工复位。

## 已验收的控制器行为

配置见 [navigation_v1.yaml](../ros2_ws/robot_navigation/config/navigation_v1.yaml)。
Humble DWB（`dwb_core::DWBLocalPlanner`）的 `FollowPath.forward_prune_distance`
从 2.0 m 调整为 0.4 m。此前局部评分终点位于薄墙另一侧，虽有正确绕墙全局路径，仍在
拐角犹豫；缩短局部路径后，评分终点保留在当前接近拐角的路段。局部代价地图始终保留
墙体，footprint 符合冻结几何，两者均无需调参。操作者确认已验收实车运行中问题不再出现。

导航上限仍为 0.20 m/s 和 0.60 rad/s。原生 Nav2 负责障碍、局部控制和 Wait 恢复；
现有运动门控保留健康检查、指令新鲜度及 STOP／故障锁存，STM32 负责执行器安全。
独立锁存式遇障停车 demo 使用单独的启动入口。

原始 rosbag 和实验日志保留在实机。跨域里程碑见 [CHANGELOG](../CHANGELOG.md)，
运行 demo 无需下载这些实验记录。
