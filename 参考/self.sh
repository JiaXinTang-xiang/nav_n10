#! /bin/bash

sleep 2

source /opt/ros/noetic/setup.bash
source /home/uu20/catkin_ws/devel/setup.bash
source /home/uu20/anorosdt_ws/devel/setup.bash
#source /home/uu20/cartographer_ws/install_isolated/setup.bash

sleep 2

#打开新终端并执行命令：gnome-terminal -- bash -c "你想执行的命令 ; exec bash"
gnome-terminal -- bash -c "roslaunch anorosdt cartographer_imu_dt.launch;exec bash" 

#多个launch文件启动，中间需要添加延时函数：sleep 5 延时5s
#sleep 5

#gnome-terminal -- bash -c "roslaunch cartographer_ros demo_revo_lds_imu.launch"
