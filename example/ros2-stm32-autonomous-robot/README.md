# ros2-stm32-autonomous-robot

[简体中文](README.zh-CN.md)

A real differential-drive robot that connects ROS 2 navigation to STM32 wheel control
through CAN FD. The system integrates LiDAR mapping, saved-map localization, path
planning and encoder feedback to drive autonomously to user-selected destinations.

## Real autonomous navigation

**Validated on hardware — 2026-09-13.** The operator confirmed multiple successful
full-map routes and thin-wall turns. All six recorded navigation goals reached
`SUCCEEDED`; the robot stopped and withdrew motion authority at arrival.

```text
Saved map → initial localization → RViz goal → path planning → autonomous travel → arrival
```

- **Mapping and localization:** RPLIDAR A1 and Cartographer build and reload the map;
  the operator sets the initial pose and checks scan-to-map alignment in RViz2.
- **Native navigation:** Nav2 plans the global route and DWB follows it locally,
  including routes around thin walls and corners. Navigation speed is capped at 0.20 m/s.
- **Drivetrain feedback:** cumulative wheel encoder positions feed differential-drive
  odometry and `odom → base_link`. Independent left/right scales preserve the
  drivetrain calibration; cumulative counts tolerate telemetry gaps.
- **Controlled execution:** the Orange Pi sends motion requests over CAN FD; STM32
  performs wheel-speed control and actuator safety supervision. Motion Authority,
  command freshness, watchdogs and fault handling remain part of the execution path.

The VM runs RViz2 for the map, live scan, robot pose and global path. On the robot,
localization starts through its service; Nav2 waits for a valid initial pose and then
an explicit goal. Sending a goal initiates motion when the health checks pass.

Acceptance covers point-to-point navigation **after initial localization**.
Integrated obstacle/safety scenarios and a complete reboot-to-goal cycle have
separate acceptance criteria. See [navigation operation and setup](docs/rviz_vm.md).

## Hardware and software

| Component | Role |
|---|---|
| Orange Pi AI Pro 8GB · Ubuntu 22.04 · ROS 2 Humble | Cartographer, Nav2, odometry and CAN bridge |
| STM32G474RET6 · FreeRTOS | Closed-loop wheel control, telemetry and actuator safety |
| RPLIDAR A1 | 2D laser mapping and navigation input |
| TB6612 · dual motors · quadrature encoders | Differential-drive actuation and wheel feedback |
| CAN FD · SocketCAN `can3` · Protocol 1.0 | Bidirectional control and telemetry, 500 kbit/s nominal / 2 Mbit/s data |

ROS determines the route and requested velocity; STM32 controls the wheels and
whether actuator motion is permitted. The [shared protocol](interfaces/protocol_v1.md)
keeps these responsibilities explicit.

## Explore and run

```bash
git clone https://github.com/LingZhen07/ros2-stm32-autonomous-robot.git
cd ros2-stm32-autonomous-robot
```

| Resource | What to find |
|---|---|
| [Navigation and RViz](docs/rviz_vm.md) | Robot setup, VM visualization, initial pose and goal operation |
| [Mapping and localization](docs/slam_mapping.md) | Cartographer configuration and saved-map workflow |
| [ROS bridge](docs/ros_bridge.md) | CAN integration, wheel odometry and ROS interfaces |
| [STM32 firmware](docs/firmware.md) | Real-time control architecture and build instructions |
| [Hardware baseline](docs/hardware.md) | Wiring, peripheral allocation and robot geometry |
| [Documentation index](docs/README.md) | Full English and Chinese technical references |

The [saved map and localization state](ros2_ws/slam_results) describe the project's
test venue. A different environment requires a corresponding map and initial pose.

## License

Copyright (c) 2026 Ling Zhen. Licensed under the [MIT License](LICENSE).
