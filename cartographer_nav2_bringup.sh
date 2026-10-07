#!/usr/bin/env bash
# Start Nav2 over an already running Cartographer pure-localization stack.
# This script deliberately does not start AMCL or map_server.

set -euo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${WS}/cyclonedds/cyclonedds-jetson.xml"

if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "❌ 未发现 /map。请先启动 cartographer_localization_bringup.sh 并在 RViz 设置初始位姿。"
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/amcl'; then
  echo "❌ 检测到 AMCL，停止启动，避免 AMCL 与 Cartographer 同时发布 map→odom。"
  exit 1
fi

cartographer_count="$(ros2 node list 2>/dev/null | grep -xc '/cartographer_node' || true)"
if [ "${cartographer_count}" -ne 1 ]; then
  echo "❌ 当前 Cartographer 节点数量为 ${cartographer_count}，必须先保证只有一个 /cartographer_node。"
  exit 1
fi

for pattern in controller_server planner_server behavior_server bt_navigator \
  velocity_smoother lifecycle_manager_navigation; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 1

echo "=== 检查定位 TF 时间戳 ==="
ros2 run wheeltec_nav2 wait_for_localization.py \
  --ros-args -p max_age_sec:=0.7 -p timeout_sec:=30.0 -p stable_samples:=5

echo "✅ 启动 Cartographer 纯定位模式 Nav2（不启动 AMCL/map_server）"
exec ros2 launch wheeltec_nav2 cartographer_navigation.launch.py rviz:=false
