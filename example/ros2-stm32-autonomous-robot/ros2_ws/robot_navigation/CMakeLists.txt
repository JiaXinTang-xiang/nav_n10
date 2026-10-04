cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_stm32_bridge REQUIRED)

add_executable(navigation_motion_gate src/navigation_motion_gate.cpp)
target_compile_features(navigation_motion_gate PUBLIC cxx_std_17)
target_compile_options(navigation_motion_gate PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(navigation_motion_gate
  action_msgs geometry_msgs rclcpp rclcpp_action nav_msgs nav2_msgs
  std_msgs std_srvs sensor_msgs tf2_ros robot_stm32_bridge)
install(TARGETS navigation_motion_gate RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY behavior_trees config launch systemd
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/start_production_navigation.sh scripts/wait_for_localization.py
  scripts/start_rviz.sh scripts/install_rviz_user.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
