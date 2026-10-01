#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('lslidar_driver')
    config_dir = os.path.join(pkg_dir, 'config')
    rviz_dir = os.path.join(pkg_dir, 'rviz')
    params_dir = os.path.join(pkg_dir, 'params', 'lidar_uart_ros2', 'lsn10.yaml')

    # 飞控桥接包路径
    anorosdt2_dir = get_package_share_directory('anorosdt2')
    anorosdt2_config = os.path.join(anorosdt2_dir, 'config', 'anorosdt2.yaml')

    # URDF 内联定义（LSN10 雷达 + IMU）
    robot_description = '<?xml version="1.0"?>' \
        '<robot name="lsn10_robot">' \
        '<link name="base_link"><visual><geometry><box size="0.5 0.5 0.2"/></geometry></visual></link>' \
        '<link name="imu_link"/>' \
        '<link name="laser"/>' \
        '<joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/></joint>' \
        '<joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/></joint>' \
        '</robot>'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 雷达驱动节点
        LifecycleNode(
            package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            output='screen',
            emulate_tty=True,
            namespace='',
            parameters=[
                params_dir,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        # 机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'lsn10.lua'
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', '/imu/data'),
            ],
            output='screen'
        ),

        # 占据栅格地图节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'resolution': 0.05}
            ],
            output='screen'
        ),

        # RViz 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(rviz_dir, 'lsn10_cartographer.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # 飞控 IMU 桥接节点（ANO PT v7 串口协议）
        Node(
            package='anorosdt2',
            executable='anoros_dt',
            name='anoros_dt',
            parameters=[anorosdt2_config],
            output='screen'
        )
    ])
