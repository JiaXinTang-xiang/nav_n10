import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def package_launch(package_name, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            package_launch("robot_bringup", "sensor_transforms.launch.py"),
            package_launch(
                "sllidar_ros2",
                "sllidar_a1_launch.py",
                {
                    "serial_port": "/dev/ttyUSB0",
                    "serial_baudrate": "115200",
                    "frame_id": "laser_frame",
                },
            ),
            package_launch(
                "robot_slam",
                "cartographer_localization.launch.py",
                {
                    "load_state_filename": (
                        "/data/projects/maps/cartographer_lidar_slam_0.pbstream"
                    ),
                    "initial_pose_from_rviz": "true",
                },
            ),
        ]
    )
