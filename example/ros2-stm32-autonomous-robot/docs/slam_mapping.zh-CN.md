# 真实激光雷达建图

[English](slam_mapping.md)

## 状态

已于 2026-09-04 在真实机器人上完成 `VERIFIED` 验收。生产 ROS 路径复用现有
Cartographer 2D，输入真实 RPLIDAR A1 扫描和已经验收的轮式里程计。

```text
/scan + /odom + TF
  -> Cartographer 2D
  -> map -> odom
  -> 保存 pbstream + PGM/YAML 地图
```

该建图里程碑无需修改 Protocol 1.0 或固件。当前生产固件为 0.5.5；基于保存地图的
全图及薄墙导航已于 2026-09-13 通过。

## 生产运行路径

建图启动命令为：

```bash
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
ros2 launch robot_slam cartographer_mapping.launch.py
```

现有 RPLIDAR 启动路径必须在 `laser_frame` 中提供 `/scan`，生产桥接节点必须提供
`/odom`，项目传感器 TF 必须提供机器人到传感器的固定变换。Cartographer 同时订阅
`/scan` 与 `/odom`，关键设置如下：

```text
map_frame:            map
tracking_frame:       base_link
published_frame:      odom
provide_odom_frame:   false
use_odometry:         true
```

建图期间的 TF 所有权严格划分如下：

| 变换 | 唯一发布者 |
|---|---|
| `map -> odom` | `/cartographer_node` |
| `odom -> base_link` | `/robot_stm32_bridge` |
| `base_link -> laser_frame` | `/base_to_laser_frame` 静态发布者 |

Cartographer 不得发布 `odom -> base_link`。结束 Cartographer 轨迹后，实时
`map -> odom` 会停止发布，这是建图结束后的正常行为。

## 已验收真实地图

机器人保持 DISARMED，由人工推动覆盖完整的大写 H 形固定场地。已验收的 0 号地图运行
使用生产 `/odom` 与 `/scan`，共生成 858 个轨迹节点和 25 个子图。最后观察到的约束修正
通常约为 0.01-0.03 m 和 0.003-0.021 rad。结果清晰覆盖 H 的两条长边、中间连接段和各端点，
主体墙线连续，未出现明显整体重影或发散。

Orange Pi 上的生产地图文件：

```text
/data/projects/maps/cartographer_lidar_slam_0.pbstream
/data/projects/maps/cartographer_lidar_slam_0.pgm
/data/projects/maps/cartographer_lidar_slam_0.yaml
```

导出占据栅格为 152 x 150，分辨率 0.05 m/格，原点为
`[-4.913818, -6.402225, 0.0]`。`cartographer_pbstream_to_ros_map` 成功读取 3.4 MiB 的完成态
pbstream，并生成有效的 Netpbm PGM 与 YAML。只保留 0 号生产地图；已按用户要求于
2026-09-06 删除 `/data/projects/maps` 和仓库 `ros2_ws/slam_results` 中的旧 v1/v2 地图文件。

生产 `.pbstream`、`.pgm`、`.yaml` 已包含在 [slam_results](../ros2_ws/slam_results) 中。
在本场地使用时复制到上述运行路径；其他环境需建立对应地图。

## 生产定位

基于 0 号地图的真实定位已完成 `VERIFIED` 验收。日常运行由 `robot-localization.service`
启动定位，用户在 RViz 显式设置 **2D Pose Estimate**；定位有效后，
`robot-navigation.service` 激活 Nav2。见[部署与操作](rviz_vm.zh-CN.md)。

仅在定位／导航服务未运行、需要手动启动定位时，使用接近机器人实际位置的地图初始位姿：

```bash
ros2 launch robot_slam cartographer_localization.launch.py \
  initial_x:=0.5041528908272049 \
  initial_y:=-1.1603667887744025 \
  initial_yaw:=1.7325219882875
```

默认值是本次建图终点；机器人放置在其他位置时必须同时覆盖这三个参数。启动文件会把地图坐标位姿
转换为 Cartographer `StartTrajectory` 所需的相对轨迹位姿，加载
`/data/projects/maps/cartographer_lidar_slam_0.pbstream` 并启动纯定位。由于 H 形场地存在重复几何，
已禁用全图回退匹配，使定位保持在显式给出的初始位姿附近，避免误选另一条 H 分支。

真实 DISARMED 验收期间，`/scan` 与 `/odom` 持续有效，本地冻结地图约束分数约为 65-80%。操作人员
手动平移并向左转动机器人；RViz 中位姿和扫描连续跟随，未出现地图跳变、错误朝向、明显发散，
停止后定位继续稳定可用。

定位期间 TF 保持单一权属：

| 变换 | 唯一发布者 |
|---|---|
| `map -> odom` | `/cartographer_node` |
| `odom -> base_link` | `/robot_stm32_bridge` |
| `base_link -> laser_frame` | `/base_to_laser_frame` 静态发布者 |

## 已观察限制

- 一个 H 端点附近仍有少量稀疏噪点，但 v2 中的大面积阳光扇形伪影已不再出现在 0 号地图，
  且未观察到实质性的墙体重影。
- 验收时 Orange Pi 根文件系统已满。通过将 ROS 日志和临时文件放到 `/data/projects`
  完成了建图。运行后恢复中清理了 1.3 GB 可丢弃的 pip 下载缓存，根分区恢复为剩余
  1.1 GB（已用 96%）；应继续把 ROS 日志/临时文件放在 `/data` 并监控根分区余量。
- 轮径比例和轮距仍属于调试标定值。只有后续定位/导航暴露出实质精度问题时再细化。
- 必须提供足够接近的地图初始位姿。为避免 H 形歧义而禁用全局匹配后，错误分支不会自动完成全局纠正。
- 手推滑移转向可能产生轮胎侧滑；本次验收中，激光定位的左转与真实运动一致，但纯轮式航向不能代表
  这次受外力推动的转向。

0 号地图上的 Nav2 点到点导航已通过：记录中的六个目标均 SUCCEEDED，操作者确认全图及薄墙
运行成功。独立建图／定位 launch 不启动 Nav2，生产导航服务负责该阶段。
集成遇障／安全场景仍需单独验收。
