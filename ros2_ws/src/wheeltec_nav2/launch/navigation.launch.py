#!/usr/bin/env python3
"""
wheeltec 导航系统一键启动
= 雷达 + 底盘 + robot_state_publisher + map_server + AMCL + Nav2 + RViz

使用:
  建图后: ros2 launch wheeltec_nav2 navigation.launch.py map:=/path/to/my_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

import lifecycle_msgs.msg


def generate_launch_description():
    nav2_pkg = get_package_share_directory('wheeltec_nav2')
    chassis_pkg = get_package_share_directory('wheeltec_chassis')
    lidar_pkg = get_package_share_directory('lslidar_driver')
    nav2_params = os.path.join(nav2_pkg, 'config', 'nav2_params.yaml')

    # ── 参数 ──
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='地图 YAML 文件路径 (必需)')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='启动 RViz')

    use_sim_time = 'false'
    autostart = 'true'

    # ── 雷达 ──
    lidar_node = LifecycleNode(
        package='lslidar_driver', executable='lslidar_driver_node',
        name='lslidar_driver_node', namespace='', output='screen', emulate_tty=True,
        parameters=[os.path.join(lidar_pkg, 'params', 'lidar_uart_ros2', 'lsn10.yaml'),
                    {'use_sim_time': False}],
    )

    # ── URDF + robot_state_publisher ──
    robot_description = (
        '<?xml version="1.0"?>'
        '<robot name="wheeltec_robot">'
        '<link name="base_link"><visual><geometry><box size="0.3 0.2 0.15"/></geometry></visual></link>'
        '<link name="imu_link"/>'
        '<link name="laser"/>'
        '<joint name="imu_joint" type="fixed"><parent link="base_link"/><child link="imu_link"/>'
        '<origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/></joint>'
        '<joint name="laser_joint" type="fixed"><parent link="base_link"/><child link="laser"/>'
        '<origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/></joint>'
        '</robot>')
    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen')

    # ── 底盘桥接 ──
    chassis_node = Node(
        package='wheeltec_chassis', executable='chassis_bridge',
        name='chassis_bridge',
        parameters=[os.path.join(chassis_pkg, 'config', 'chassis.yaml')],
        output='screen')

    # ── 地图服务器 ──
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': False}],
        output='screen')

    # ── AMCL 定位 ──
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=[nav2_params], output='screen',
        remappings=[('scan', '/scan')])

    # ── Nav2 生命周期节点 ──
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': lifecycle_nodes}])

    # 各 Nav2 组件
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/cmd_vel')])

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[nav2_params])

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[nav2_params])

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[nav2_params])

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')])

    # ── RViz ──
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav2_pkg, 'config', 'nav2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        map_arg,
        rviz_arg,
        lidar_node,
        robot_state_pub,
        chassis_node,
        map_server_node,
        amcl_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
        rviz_node,
    ])
