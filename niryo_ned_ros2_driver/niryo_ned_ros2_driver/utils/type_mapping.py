# /usr/bin/env python3

import re
from .constants import ROS1_INTERFACE_PACKAGES, ROS2_INTERFACE_PACKAGE


def convert_ros1_to_ros2_type(ros1_type: str, interface_type: str) -> str:
    if interface_type not in ["srv", "msg", "action"]:
        raise ValueError(
            f"Invalid interface type '{interface_type}'. Expected 'srv', 'msg' or 'action'."
        )

    pkg, type = ros1_type.split("/")
    if interface_type == "action":
        type = type.replace("Action", "")

    if pkg in ROS1_INTERFACE_PACKAGES:
        return f"{ROS2_INTERFACE_PACKAGE}/{interface_type}/{type}"
    else:
        return f"{pkg}/{interface_type}/{type}"


def guess_action_type_from_goal_type(goal_type: str) -> str:
    """
    Given a goal type, return the corresponding action type.
    """
    pattern = r"(?P<package>.+)/(?P<basename>.+)ActionGoal"
    match = re.match(pattern, goal_type)
    if not match:
        raise ValueError(f"Provided type {goal_type} is not an ActionGoal type.")

    package = match.group("package")
    basename = match.group("basename")
    return f"{package}/{basename}Action"
