cmake_minimum_required(VERSION 3.8)
project(robot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch systemd
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_localization_from_pose.py
  scripts/start_production_localization.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
