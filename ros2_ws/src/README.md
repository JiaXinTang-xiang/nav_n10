# lslidar

## Description
The `lslidar package is a linux ROS2 driver for lslidar M10 ,M10_GPS,M10_P,M10_PLUS and N10.
The package is tested on Ubuntu 20.04 with ROS2 indigo.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.


cd /home/jiaxintang/wheeltec_ros2
source install/setup.bash
ros2 launch lslidar_driver lsn10_cartographer.launch.py

