# RViz2 and real navigation

[中文](rviz_vm.zh-CN.md)

**PASS — 2026-09-13:** the operator confirmed successful full-map and thin-wall
navigation. Six recorded explicit goals all reached `SUCCEEDED`, with no progress
failure and final motion-authority withdrawal. This covers point-to-point motion
after initial localization. Integrated obstacle/safety scenarios and a complete
reboot-to-goal cycle require separate acceptance.

## Run RViz in the Ubuntu VM

The Orange Pi runs ROS, localization and Nav2; the VM runs RViz only. Use VMware
**Bridged** networking on the same LAN as `robot-core.local`. From the repository
root inside the VM:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
rviz2 -d ./ros2_ws/robot_navigation/config/navigation.rviz
```

Use a Linux path accessible inside the VM. To install the **Robot Navigation RViz**
launcher and login autostart entry, run once:

```bash
bash ./ros2_ws/robot_navigation/scripts/install_rviz_user.sh
```

The installer backs up existing RViz defaults. Desktop-login autostart is configured;
manual launcher operation has been verified.

## Operate the demo

1. Keep the robot stationary and open the saved map in RViz.
2. Use **2D Pose Estimate** at the actual robot position, dragging toward its heading.
3. Confirm the purple scan aligns with the map walls. Nav2 activates after localization.
4. Use **2D Goal Pose** to select a destination and heading. **Sending a goal starts
   autonomous motion** when the existing health checks pass.
5. The robot follows the green path, stops at arrival and withdraws motion authority.

| Display | Topic |
|---|---|
| Map | `/map` |
| Purple laser scan | `/scan` |
| Robot odometry | `/odom` |
| Green global path | `/plan` |
| Robot footprint | `/local_costmap/published_footprint` |

Fixed Frame is `map`. Before initial localization, map-aligned scan and robot pose
may be unavailable. Cartographer owns `map -> odom`, the bridge owns
`odom -> base_link`, and the sensor transform node owns `base_link -> laser_frame`.
Do not start duplicate localization or TF publishers.

From a ROS Humble terminal on the same ROS domain:

```bash
ros2 service call /navigation_safety_gate/stop std_srvs/srv/Trigger '{}'
# Only after resolving the STOP/fault cause; reset never restores an old goal:
ros2 service call /navigation_safety_gate/reset std_srvs/srv/Trigger '{}'
```

STOP/faults persist across restarts. Goals are explicit and volatile; they are never
replayed. Use `/goal_pose`, not direct Nav2 action or bridge-arm calls that bypass
the existing motion boundary.

## Orange Pi setup

The ROS workspace is `/data/ros2_ws`; runtime Python is `/usr/bin/python3`.
See [bridge setup](ros_bridge.md) and [mapping/localization](slam_mapping.md).
The saved map in `ros2_ws/slam_results/` belongs to the project test venue. For that
venue, copy its `.pbstream`, `.pgm` and `.yaml` files into `/data/projects/maps/`.
For a different venue, create a corresponding map and initialize the actual pose.

After building and installing `robot_slam` and `robot_navigation`, install their
service definitions once on the Orange Pi:

```bash
sudo install -m 0644 /data/ros2_ws/install/robot_slam/share/robot_slam/systemd/robot-localization.service /etc/systemd/system/
sudo install -m 0644 /data/ros2_ws/install/robot_navigation/share/robot_navigation/systemd/robot-navigation.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-localization.service robot-navigation.service
```

The bridge service remains separate. Localization starts the LiDAR, sensor TF and
Cartographer, then waits for `/initialpose`. The navigation service waits for valid
localization before activating Nav2. Service activation and the initial-pose handoff
are verified; they do not imply acceptance of a full reboot-to-goal sequence.

For manual operation **only when the navigation service is not running**, reuse
already running sensors and initialized localization:

```bash
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
ros2 launch robot_navigation navigation_v1.launch.py start_localization_runtime:=false
```

After a STOP/fault, preserve the latch during a maintenance launch with
`startup_locked:=true`; reset manually only after health is restored.

## Accepted controller behavior

Configuration: [navigation_v1.yaml](../ros2_ws/robot_navigation/config/navigation_v1.yaml).
Humble DWB (`dwb_core::DWBLocalPlanner`) uses `FollowPath.forward_prune_distance=0.4` m,
reduced from 2.0 m. Previously the local scoring endpoint lay across a thin wall,
causing corner hesitation despite a correct global detour. The shorter local path
keeps the endpoint on the current approach. The wall remained present in the local
costmap and the footprint matched the frozen geometry; neither required tuning.
The operator confirmed the issue no longer occurred in the accepted physical runs.

Navigation limits remain 0.20 m/s and 0.60 rad/s. Nav2 owns obstacles, local control
and native Wait recovery. The existing motion boundary retains health checks,
command freshness and latched STOP/fault handling; STM32 remains authoritative for
actuator safety. The standalone latched obstacle-stop demo is a separate launch.

Raw bags and experiment logs stay on the robot. Shared milestones are recorded in
[CHANGELOG](../CHANGELOG.md); they are not required to run the demo.
