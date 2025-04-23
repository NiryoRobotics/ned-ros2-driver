# /usr/bin/env python3

from .constants import ROS1_INTERFACE_PACKAGES, ROS2_INTERFACE_PACKAGE


def convert_ros1_to_ros2_type(ros1_type: str, interface_type: str) -> str:
    if interface_type not in ["srv", "msg"]:
        raise ValueError(
            f"Invalid interface type '{interface_type}'. Expected 'srv' or 'msg'."
        )

    pkg, type = ros1_type.split("/")
    if pkg in ROS1_INTERFACE_PACKAGES:
        return f"{ROS2_INTERFACE_PACKAGE}/{interface_type}/{type}"
    else:
        return f"{pkg}/{interface_type}/{type}"
