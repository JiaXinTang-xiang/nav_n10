# Real LiDAR Mapping

[中文](slam_mapping.zh-CN.md)

## Status

`VERIFIED` on the real robot on 2026-09-04. The production ROS path uses the existing
Cartographer 2D stack with real RPLIDAR A1 scans and the accepted wheel odometry.

```text
/scan + /odom + TF
  -> Cartographer 2D
  -> map -> odom
  -> saved pbstream + PGM/YAML map
```

The mapping milestone required no firmware or Protocol 1.0 change. Production now
uses firmware 0.5.5; map-based full-map and thin-wall navigation passed on 2026-09-13.

## Production runtime

The mapping launch is:

```bash
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
ros2 launch robot_slam cartographer_mapping.launch.py
```

The existing RPLIDAR launch must provide `/scan` in `laser_frame`, the production bridge must
provide `/odom`, and the project sensor transforms must provide the fixed robot-to-sensor links.
The Cartographer configuration consumes both `/scan` and `/odom` and uses:

```text
map_frame:            map
tracking_frame:       base_link
published_frame:      odom
provide_odom_frame:   false
use_odometry:         true
```

TF ownership during mapping is intentionally split as follows:

| Transform | Sole authority |
|---|---|
| `map -> odom` | `/cartographer_node` |
| `odom -> base_link` | `/robot_stm32_bridge` |
| `base_link -> laser_frame` | `/base_to_laser_frame` static publisher |

Cartographer must not publish `odom -> base_link`. Finishing the Cartographer trajectory stops
the live `map -> odom` transform; this is expected after mapping has ended.

## Accepted real map

The robot was manually moved while DISARMED through the complete H-shaped fixed venue. During
the accepted map 0 run, Cartographer received the production `/odom` and `/scan`, generated 858
trajectory nodes in 25 submaps, and closed constraints with small corrections (typically about
0.01-0.03 m and 0.003-0.021 rad in the final observed matches). The result clearly shows both
long parts of the H, the middle connection, and coherent primary wall lines without gross
duplication or divergence.

Production artifacts on the Orange Pi:

```text
/data/projects/maps/cartographer_lidar_slam_0.pbstream
/data/projects/maps/cartographer_lidar_slam_0.pgm
/data/projects/maps/cartographer_lidar_slam_0.yaml
```

The exported occupancy grid is 152 x 150 cells at 0.05 m/cell with origin
`[-4.913818, -6.402225, 0.0]`. `cartographer_pbstream_to_ros_map` successfully read the finished
3.4 MiB pbstream and generated a valid Netpbm PGM plus YAML metadata. Map 0 is the only
retained production map; obsolete v1/v2 artifacts were deleted at the operator's request
on 2026-09-06 from `/data/projects/maps` and the repository's `ros2_ws/slam_results`.

The production `.pbstream`, `.pgm` and `.yaml` are now included in
[slam_results](../ros2_ws/slam_results). Copy them to the runtime paths above for
this venue; another environment requires its own map.

## Production localization

Real localization against map 0 is `VERIFIED`. Normal operation uses
`robot-localization.service` and an explicit RViz **2D Pose Estimate**;
`robot-navigation.service` activates Nav2 after localization becomes valid.
See [deployment and operation](rviz_vm.md).

For manual localization only, with the localization/navigation services not running,
start the existing path with a map-frame initial pose close to the actual placement:

```bash
ros2 launch robot_slam cartographer_localization.launch.py \
  initial_x:=0.5041528908272049 \
  initial_y:=-1.1603667887744025 \
  initial_yaw:=1.7325219882875
```

Those defaults are the accepted mapping endpoint. Override all three values whenever the robot is
placed elsewhere. The launch converts the map-frame pose to Cartographer's trajectory-relative
`StartTrajectory` convention, loads
`/data/projects/maps/cartographer_lidar_slam_0.pbstream`, and starts pure localization. Because the
H venue contains repeated geometry, full-map fallback matching is disabled; localization stays
near the explicitly supplied initial pose instead of selecting an ambiguous H branch.

During real DISARMED validation, `/scan` and `/odom` remained live and local frozen-map constraint
scores were approximately 65-80%. The operator manually translated and turned the robot left;
the RViz pose and scan followed continuously with no map jump, wrong heading, gross divergence,
or loss of the stationary pose afterward.

TF ownership in localization remains single-authority:

| Transform | Sole authority |
|---|---|
| `map -> odom` | `/cartographer_node` |
| `odom -> base_link` | `/robot_stm32_bridge` |
| `base_link -> laser_frame` | `/base_to_laser_frame` static publisher |

## Observed limitations

- A small amount of sparse noise remains near one H end, but the large sunlight-induced fan in
  v2 is absent from map 0 and no material wall duplication is visible.
- The Orange Pi root filesystem was full during acceptance. Mapping succeeded by placing ROS
  logs and temporary files under `/data/projects`. Post-run recovery purged 1.3 GB of disposable
  pip download cache, leaving 1.1 GB free (96% used); keep ROS logs/temporary files on `/data` and
  monitor the remaining root capacity.
- Wheel scale and track-width values remain commissioning calibration. Refine them only if the
  localization/navigation stage exposes a material accuracy problem.
- A reasonably close initial map pose is required. Intentional suppression of ambiguous global
  matching means a wrong H branch is not expected to self-correct automatically.
- Hand-pushing a skid-steer turn can introduce wheel slip; during acceptance, the LiDAR-localized
  left turn followed physical motion while wheel-only yaw did not represent that pushed turn.

Nav2 point-to-point navigation against map 0 is accepted: six recorded goals reached
SUCCEEDED, and the operator confirmed successful full-map and thin-wall runs.
The standalone mapping/localization launches do not start Nav2; the production
navigation service supplies that stage. Integrated obstacle/safety acceptance remains separate.
