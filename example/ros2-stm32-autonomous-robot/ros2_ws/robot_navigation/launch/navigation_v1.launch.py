import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def package_launch(package_name, launch_file, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package_name), "launch", launch_file)
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    package_share = get_package_share_directory("robot_navigation")
    navigation_params = os.path.join(package_share, "config", "navigation_v1.yaml")
    safe_bt = os.path.join(
        package_share, "behavior_trees", "navigate_to_pose_safe.xml"
    )

    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    sensor_transforms = package_launch(
        "robot_bringup", "sensor_transforms.launch.py"
    )
    rplidar = package_launch(
        "sllidar_ros2",
        "sllidar_a1_launch.py",
        {
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "115200",
            "frame_id": "laser_frame",
        },
    )
    localization = package_launch(
        "robot_slam",
        "cartographer_localization.launch.py",
        {
            "load_state_filename": (
                "/data/projects/maps/cartographer_lidar_slam_0.pbstream"
            ),
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
            "initial_pose_from_rviz": LaunchConfiguration("initial_pose_from_rviz"),
        },
    )

    safety_gate = Node(
        package="robot_navigation",
        executable="navigation_motion_gate",
        name="navigation_safety_gate",
        parameters=[navigation_params, {
            "startup_locked": LaunchConfiguration("startup_locked"),
            "latch_file": ParameterValue(LaunchConfiguration("latch_file"), value_type=str),
        }],
        output="screen",
    )
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[navigation_params],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        output="screen",
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[navigation_params],
        output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[navigation_params],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        output="screen",
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        # The existing safety gate now owns /goal_pose and sends Nav2 actions.
        # Do not let the navigator's convenience topic subscriber send a duplicate goal.
        remappings=[("goal_pose", "/_nav2_direct_goal_disabled")],
        parameters=[
            navigation_params,
            {
                "default_nav_to_pose_bt_xml": safe_bt,
                "default_nav_through_poses_bt_xml": os.path.join(
                    package_share, "behavior_trees", "navigate_through_poses_safe.xml"
                ),
            },
        ],
        output="screen",
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "latch_file",
                default_value="",
                description="Persistent host STOP/fault latch for the production service",
            ),
            DeclareLaunchArgument(
                "startup_locked",
                default_value="false",
                description="Preserve STOP/fault latch through a maintenance restart; reset remains manual",
            ),
            DeclareLaunchArgument(
                "start_localization_runtime",
                default_value="true",
                description="Start sensors, static TF and localization; false reuses an already initialized runtime",
            ),
            DeclareLaunchArgument(
                "initial_pose_from_rviz",
                default_value="true",
                description="Require one explicit RViz 2D Pose Estimate before localization",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.5041528908272049",
                description="Initial base_link X in map 0",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="-1.1603667887744025",
                description="Initial base_link Y in map 0",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="1.7325219882875",
                description="Initial base_link yaw in map 0",
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration("start_localization_runtime")),
                actions=[sensor_transforms, rplidar, localization],
            ),
            safety_gate,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )
