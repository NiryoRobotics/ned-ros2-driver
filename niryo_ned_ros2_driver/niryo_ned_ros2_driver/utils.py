# /usr/bin/env python3


def convert_ros1_to_ros2_type(ros1_type: str, ros2_interfaces_package: str) -> str:
    """
    Convert a custom ROS1 type to its corresponding ROS2 type.
    """
    return ros2_interfaces_package + "/" + ros1_type.rsplit("/", 1)[-1]
