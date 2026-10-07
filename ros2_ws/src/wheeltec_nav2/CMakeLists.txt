cmake_minimum_required(VERSION 3.8)
project(wheeltec_nav2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

pluginlib_export_plugin_description_file(nav2_core imu_behavior_plugin.xml)

include_directories(include)

add_library(wheeltec_imu_spin_behavior SHARED
  src/imu_spin.cpp
)
ament_target_dependencies(wheeltec_imu_spin_behavior
  geometry_msgs
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  sensor_msgs
  tf2
)

add_library(wheeltec_nav2_bt_nodes SHARED
  src/final_imu_spin_action.cpp
)
target_compile_definitions(wheeltec_nav2_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(wheeltec_nav2_bt_nodes
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  tf2
)

install(TARGETS wheeltec_imu_spin_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS wheeltec_nav2_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES imu_behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES bt_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigation_autostarter.py
  scripts/start_cartographer_localization.py
  scripts/wait_for_localization.py
  scripts/imu_spin_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
