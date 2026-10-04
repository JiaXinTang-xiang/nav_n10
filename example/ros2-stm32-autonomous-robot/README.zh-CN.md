# ros2-stm32-autonomous-robot

[English](README.md)

一套将 ROS 2 导航与 STM32 底盘控制通过 CAN FD 连通的实车差速机器人。
系统集成激光建图、保存地图定位、路径规划与编码器反馈，实现从用户指定目标到自主行驶、到达停车的完整链路。

## 实车自主导航

**实车验证通过 — 2026-09-13。** 操作者确认多次全图导航和薄墙绕行均成功；
记录中的六个导航目标全部返回 `SUCCEEDED`，机器人到达后停车并撤销运动授权。

```text
保存地图 → 初始定位 → RViz 指定目标 → 路径规划 → 自主行驶 → 到达停车
```

- **建图与定位：** RPLIDAR A1 与 Cartographer 完成激光建图和地图重载；操作者在 RViz2
  设置初始位姿，确认扫描与地图对齐后开始导航。
- **原生导航：** Nav2 规划全局路线，DWB 执行局部路径跟踪，完成薄墙绕行和拐角通过。
  当前导航速度上限为 0.20 m/s。
- **底盘反馈：** 由左右轮累计编码器位置计算差速里程计，发布 `odom → base_link`。
  保留左右轮独立尺度，累计位置反馈可容忍遥测丢帧。
- **受控执行：** Orange Pi 通过 CAN FD 发送运动请求，STM32 负责轮速闭环与执行器安全监督。
  运动授权、指令新鲜度、看门狗及故障处理共同约束实际运动。

VM 中的 RViz2 显示地图、实时雷达、机器人位置和全局路径。实机定位服务启动后，
Nav2 等待有效初始位姿及显式导航目标；发送目标且健康检查通过后，机器人自主出发。

验收范围为**初始定位完成后的点到点自主导航**。集成遇障／安全场景和完整重启至到达流程
另行验收。部署与操作见[导航及 RViz 使用说明](docs/rviz_vm.zh-CN.md)。

## 硬件与软件

| 组成 | 职责 |
|---|---|
| Orange Pi AI Pro 8GB · Ubuntu 22.04 · ROS 2 Humble | Cartographer、Nav2、里程计与 CAN 桥接 |
| STM32G474RET6 · FreeRTOS | 轮速闭环、遥测与执行器安全 |
| RPLIDAR A1 | 二维激光建图与导航输入 |
| TB6612 · 双电机 · 正交编码器 | 差速驱动与轮运动反馈 |
| CAN FD · SocketCAN `can3` · Protocol 1.0 | 双向控制与遥测，仲裁段 500 kbit/s、数据段 2 Mbit/s |

ROS 决定路线和请求速度；STM32 控制车轮，并决定是否允许执行运动。
两端通过[共享协议](interfaces/protocol_v1.md)明确职责边界。

## 阅读与运行

```bash
git clone https://github.com/LingZhen07/ros2-stm32-autonomous-robot.git
cd ros2-stm32-autonomous-robot
```

| 入口 | 内容 |
|---|---|
| [导航与 RViz](docs/rviz_vm.zh-CN.md) | 实机部署、VM 可视化、初始定位与目标操作 |
| [建图与定位](docs/slam_mapping.zh-CN.md) | Cartographer 配置与保存地图流程 |
| [ROS 桥接](docs/ros_bridge.zh-CN.md) | CAN 接入、轮式里程计与 ROS 接口 |
| [STM32 固件](docs/firmware.zh-CN.md) | 实时控制架构与构建说明 |
| [硬件基线](docs/hardware.zh-CN.md) | 接线、外设分配与机器人几何 |
| [文档索引](docs/README.zh-CN.md) | 完整中英文技术资料 |

[保存地图与定位状态](ros2_ws/slam_results)对应项目测试场地。
在其他环境运行时，需建立对应地图并设置实际初始位姿。

## License

Copyright (c) 2026 Ling Zhen。本项目采用 [MIT License](LICENSE)。
