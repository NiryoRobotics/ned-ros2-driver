from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_config_path = os.path.join(
        get_package_share_directory("niryo_ned_ros2_driver"), "config", "config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="niryo_ned_ros2_driver",
                executable="ros2_driver",
                name="ros2_driver",
                parameters=[default_config_path],
            )
        ]
    )
