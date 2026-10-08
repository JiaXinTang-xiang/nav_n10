#!/usr/bin/env python3
"""Start Nav2 on top of an already running Cartographer localization stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav2_pkg = get_package_share_directory('wheeltec_nav2')
    params_file = os.path.join(nav2_pkg, 'config', 'nav2_params.yaml')
    behavior_tree = os.path.join(
        nav2_pkg, 'config', 'navigate_to_pose_safe.xml')

    rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='Start RViz on Jetson')

    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/cmd_vel')])

    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[params_file])

    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[params_file])

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        remappings=[('goal_pose', '/_nav2_direct_goal_disabled')],
        parameters=[params_file, {
            'default_nav_to_pose_bt_xml': behavior_tree,
        }])

    goal_guard = Node(
        package='wheeltec_nav2', executable='navigation_goal_guard.py',
        name='navigation_goal_guard', output='screen')

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(nav2_pkg, 'config', 'nav2.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen')

    return LaunchDescription([
        rviz,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        goal_guard,
        lifecycle_manager,
        rviz_node,
    ])
