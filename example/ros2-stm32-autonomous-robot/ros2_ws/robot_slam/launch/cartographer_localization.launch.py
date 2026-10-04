from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Cartographer interprets StartTrajectory.initial_pose relative to trajectory
# zero at timestamp zero. These are the first frozen-trajectory node coordinates
# in the accepted map 0 pbstream; launch arguments remain ordinary map-frame poses.
FROZEN_TRAJECTORY_REFERENCE_X = -0.0032213622856155084
FROZEN_TRAJECTORY_REFERENCE_Y = 0.004257684416241552
FROZEN_TRAJECTORY_REFERENCE_YAW = 0.0022849634604274


def generate_launch_description():
    configuration_directory = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), "config"]
    )
    load_state_filename = LaunchConfiguration("load_state_filename")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    initial_pose_from_rviz = LaunchConfiguration("initial_pose_from_rviz")
    initial_relative_x = PythonExpression(
        [
            "str(__import__('math').cos(",
            str(FROZEN_TRAJECTORY_REFERENCE_YAW),
            ") * (float('",
            initial_x,
            "') - ",
            str(FROZEN_TRAJECTORY_REFERENCE_X),
            ") + __import__('math').sin(",
            str(FROZEN_TRAJECTORY_REFERENCE_YAW),
            ") * (float('",
            initial_y,
            "') - ",
            str(FROZEN_TRAJECTORY_REFERENCE_Y),
            "))",
        ]
    )
    initial_relative_y = PythonExpression(
        [
            "str(-__import__('math').sin(",
            str(FROZEN_TRAJECTORY_REFERENCE_YAW),
            ") * (float('",
            initial_x,
            "') - ",
            str(FROZEN_TRAJECTORY_REFERENCE_X),
            ") + __import__('math').cos(",
            str(FROZEN_TRAJECTORY_REFERENCE_YAW),
            ") * (float('",
            initial_y,
            "') - ",
            str(FROZEN_TRAJECTORY_REFERENCE_Y),
            "))",
        ]
    )
    initial_relative_yaw = PythonExpression(
        [
            "str(float('",
            initial_yaw,
            "') - ",
            str(FROZEN_TRAJECTORY_REFERENCE_YAW),
            ")",
        ]
    )
    initial_qz = PythonExpression(
        [
            "str(__import__('math').sin(float('",
            initial_relative_yaw,
            "') / 2.0))",
        ]
    )
    initial_qw = PythonExpression(
        [
            "str(__import__('math').cos(float('",
            initial_relative_yaw,
            "') / 2.0))",
        ]
    )
    start_trajectory_request = [
        "{configuration_directory: '",
        configuration_directory,
        "', configuration_basename: 'rplidar_a1_2d_localization.lua', "
        "use_initial_pose: true, initial_pose: {position: {x: ",
        initial_relative_x,
        ", y: ",
        initial_relative_y,
        ", z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ",
        initial_qz,
        ", w: ",
        initial_qw,
        "}}, relative_to_trajectory_id: 0}",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "initial_pose_from_rviz",
                default_value="false",
                description="Wait for one map-frame /initialpose instead of using coordinates",
            ),
            DeclareLaunchArgument(
                "load_state_filename",
                default_value=(
                    "/data/projects/maps/cartographer_lidar_slam_0.pbstream"
                ),
                description="Finished Cartographer state used for localization",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.5041528908272049",
                description="Initial base_link X in the accepted map frame",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="-1.1603667887744025",
                description="Initial base_link Y in the accepted map frame",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="1.7325219882875",
                description="Initial base_link yaw in the accepted map frame",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                arguments=[
                    "-configuration_directory",
                    configuration_directory,
                    "-configuration_basename",
                    "rplidar_a1_2d_localization.lua",
                    "-load_state_filename",
                    load_state_filename,
                    "-start_trajectory_with_default_topics=false",
                ],
                remappings=[("scan", "/scan"), ("odom", "/odom")],
                output="screen",
            ),
            TimerAction(
                period=1.0,
                condition=UnlessCondition(initial_pose_from_rviz),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/start_trajectory",
                            "cartographer_ros_msgs/srv/StartTrajectory",
                            start_trajectory_request,
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="robot_slam",
                executable="start_localization_from_pose.py",
                condition=IfCondition(initial_pose_from_rviz),
                parameters=[{
                    "configuration_directory": configuration_directory,
                    "reference_x": FROZEN_TRAJECTORY_REFERENCE_X,
                    "reference_y": FROZEN_TRAJECTORY_REFERENCE_Y,
                    "reference_yaw": FROZEN_TRAJECTORY_REFERENCE_YAW,
                }],
                output="screen",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                parameters=[
                    {"use_sim_time": False},
                    {"resolution": 0.05},
                ],
                output="screen",
            ),
        ]
    )
