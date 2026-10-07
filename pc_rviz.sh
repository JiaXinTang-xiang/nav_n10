#!/usr/bin/env bash
set -e

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [ -f "${project_dir}/ros2_ws/install/setup.bash" ]; then
  source "${project_dir}/ros2_ws/install/setup.bash"
fi

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${project_dir}/cyclonedds/cyclonedds-pc.xml"

mode="${1:-nav}"
case "${mode}" in
  nav)
    config="${project_dir}/ros2_ws/src/wheeltec_nav2/config/nav2.rviz"
    ;;
  slam)
    config="${project_dir}/ros2_ws/src/wheeltec_nav2/config/mapping_minimal.rviz"
    ;;
  *)
    echo "用法: $0 [nav|slam]"
    exit 1
    ;;
esac

export LIBGL_DRI3_DISABLE=1
exec rviz2 -d "${config}"
