#!/usr/bin/env python3
"""
wheeltec 导航系统一键启动
= 雷达 + 底盘 + robot_state_publisher + map_server + AMCL + Nav2 + RViz

使用:
  建图后: ros2 launch wheeltec_nav2 navigation.launch.py map:=/path/to/my_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_pkg = get_package_share_directory('wheeltec_nav2')
    chassis_pkg = get_package_share_directory('wheeltec_chassis')
    lidar_pkg = get_package_share_directory('lslidar_driver')
    imu_pkg = get_package_share_directory('anorosdt2')
    nav2_params = os.path.join(nav2_pkg, 'config', 'nav2_params.yaml')
    safe_navigation_tree = os.path.join(
        nav2_pkg, 'config', 'navigate_to_pose_safe.xml')

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
    lidar_node = Node(
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

    # ── 飞控 IMU ──
    imu_node = Node(
        package='anorosdt2', executable='anoros_dt', name='anoros_dt',
        parameters=[os.path.join(imu_pkg, 'config', 'anorosdt2.yaml')],
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

    # 定位先启动；其余导航节点在 AMCL 首次定位后自动激活。
    localization_nodes = [
        'map_server',
        'amcl',
    ]
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    localization_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': localization_nodes}])

    navigation_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': False,
                     'node_names': navigation_nodes}])

    navigation_autostarter = Node(
        package='wheeltec_nav2', executable='navigation_autostarter.py',
        name='navigation_autostarter', output='screen')

    # 各 Nav2 组件
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/cmd_vel_nav')])

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
        parameters=[nav2_params, {
            'default_nav_to_pose_bt_xml': safe_navigation_tree,
        }])

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
        imu_node,
        chassis_node,
        map_server_node,
        amcl_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        localization_manager,
        navigation_manager,
        navigation_autostarter,
        rviz_node,
    ])
