# /usr/bin/env python3

from .constants import ROS1_INTERFACE_PACKAGES, ROS2_INTERFACE_PACKAGE


def convert_ros1_to_ros2_type(ros1_type: str) -> str:
    pkg, msg = ros1_type.split("/")
    if pkg in ROS1_INTERFACE_PACKAGES:
        return f"{ROS2_INTERFACE_PACKAGE}/msg/{msg}"
    else:
        return f"{pkg}/msg/{msg}"
