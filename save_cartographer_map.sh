#!/usr/bin/env bash
# Save the active Cartographer map as both pbstream and PGM/YAML.

set -euo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${1:-${WS}/maps}"
MAP_NAME="${2:-map_$(date +%Y%m%d_%H%M%S)}"
FILE_STEM="${OUTPUT_DIR}/${MAP_NAME}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://${WS}/cyclonedds/cyclonedds-jetson.xml}"

mkdir -p "${OUTPUT_DIR}"

if ! timeout 5s ros2 service type /write_state 2>/dev/null | \
    grep -q '^cartographer_ros_msgs/srv/WriteState$'; then
  echo "❌ /write_state 不可用，请先启动 Cartographer 建图。"
  exit 1
fi

echo "=== 保存 Cartographer 状态 ==="
timeout 60s ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '${FILE_STEM}.pbstream', include_unfinished_submaps: true}"

if [ ! -s "${FILE_STEM}.pbstream" ]; then
  echo "❌ pbstream 未生成或为空: ${FILE_STEM}.pbstream"
  exit 1
fi

echo "=== 从同一 pbstream 生成 PGM/YAML ==="
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename "${FILE_STEM}.pbstream" \
  -map_filestem "${FILE_STEM}" \
  -resolution 0.05

echo "✅ 地图保存完成"
ls -lh "${FILE_STEM}.pbstream" "${FILE_STEM}.pgm" "${FILE_STEM}.yaml"
echo "PBSTREAM=${FILE_STEM}.pbstream"
