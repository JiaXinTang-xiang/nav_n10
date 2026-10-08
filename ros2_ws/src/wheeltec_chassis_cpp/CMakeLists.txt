cmake_minimum_required(VERSION 3.8)
project(wheeltec_chassis_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_bridge_cpp src/chassis_bridge.cpp)
ament_target_dependencies(chassis_bridge_cpp
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)

install(TARGETS chassis_bridge_cpp
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
