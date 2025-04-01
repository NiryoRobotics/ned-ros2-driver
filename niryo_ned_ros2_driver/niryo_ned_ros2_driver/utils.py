# /usr/bin/env python3

import fnmatch
import time
from typing import Dict

from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration


ROS2_INTERFACE_PACKAGE = "niryo_ned_ros2_interfaces"
ROS1_INTERFACE_PACKAGES = [
    "niryo_robot_arm_commander",
    "niryo_robot_tools_commander",
    "niryo_robot_vision",
    "niryo_robot_user_interface",
    "niryo_robot_blockly",
    "niryo_robot_database",
    "niryo_robot_led_ring",
    "niryo_robot_poses_handlers",
    "niryo_robot_metrics",
    "niryo_robot_msgs",
    "niryo_robot_programs_manager_v2",
    "niryo_robot_programs_manager",
    "niryo_robot_rpi",
    "niryo_robot_sound",
    "niryo_robot_status",
    "end_effector_interface",
    "joints_interface",
    "tools_interface",
    "ttl_driver",
]
BLACKLISTED_INTERFACES = [
    "/niryo_robot_programs_manager/program_list",
    "/niryo_robot_programs_manager/program_is_running",
    "/connected_clients",
    "/niryo_robot_follow_joint_trajectory_controller/state",  # TODO(Thomas): we might need to find a solution to reabilitate this topic
]


def measure_time(label: str, func, *args, **kwargs):
    """
    Utility function to measure execution time of any callable.
    """
    start = time.time()
    result = func(*args, **kwargs)
    duration = time.time() - start
    return result, duration, label


def convert_ros1_to_ros2_type(ros1_type: str) -> str:
    """
    Convert a custom ROS1 type to its corresponding ROS2 type.
    """
    pkg, msg = ros1_type.split("/")
    if pkg in ROS1_INTERFACE_PACKAGES:
        return f"{ROS2_INTERFACE_PACKAGE}/msg/{msg}"
    else:
        return f"{pkg}/msg/{msg}"


def matches_any(patterns, topic):
    """
    Returns True if topic matches any pattern in the list (supports glob-style).
    """
    return any(fnmatch.fnmatch(topic, pattern) for pattern in patterns)


def filter_topics(topic_type_map: Dict):
    """
    Efficiently filters {topic: type} pairs based on:
    - Action topic exclusion
    - Forbidden types
    - Blacklisted topics

    Parameters:
        topic_type_map (dict): Dict of {topic_name: topic_type}.

    Returns:
        dict: Filtered {topic_name: topic_type} pairs.
    """
    result = {}

    for topic, topic_type in topic_type_map.items():
        if is_action_topic(topic):
            continue
        if is_non_existing_ros2_topic_type(topic_type):
            continue
        if is_blacklisted_topic(topic):
            continue
        result[topic] = topic_type

    return result


def is_blacklisted_topic(topic):
    """
    Returns True if the topic is blacklisted.
    """
    return matches_any(BLACKLISTED_INTERFACES, topic)


def is_action_topic(topic):
    """
    Returns True if the topic is an action topic.
    """
    action_suffixes = ["/goal", "/cancel", "/status", "/result", "/feedback"]
    return any(topic.endswith(suffix) for suffix in action_suffixes)


def is_non_existing_ros2_topic_type(topic):
    """
    Returns True if the topic type does not have equivalent in ROS2.
    """
    incompatible_topic_types = [
        "dynamic_reconfigure",
        "rosgraph_msgs",
        "bond",
    ]
    return any(
        topic.startswith(incompatible) for incompatible in incompatible_topic_types
    )


def convert_dict_to_time(d: dict) -> Time:
    return Time(sec=d.get("secs", 0), nanosec=d.get("nsecs", 0))


def convert_dict_to_duration(d: dict) -> Duration:
    return Duration(sec=d.get("secs", 0), nanosec=d.get("nsecs", 0))


def convert_dict_to_header(d: dict) -> Header:
    d.pop("seq", None)
    return Header(
        stamp=convert_dict_to_time(d.get("stamp", {})), frame_id=d.get("frame_id", "")
    )


def normalize_ros1_type_to_ros2(obj: dict, ros2_type_str: str):
    """
    Applies type-specific normalization of a ROS1 message dict to fit expected ROS2 format.
    """

    def recursive_ros2_normalization(o):
        if isinstance(o, dict):
            for key, value in o.items():
                if key == "header" and isinstance(value, dict):
                    o[key] = convert_dict_to_header(value)

                elif key == "time_from_start" and isinstance(value, dict):
                    o[key] = convert_dict_to_duration(value)

                else:
                    recursive_ros2_normalization(value)

        elif isinstance(o, list):
            for item in o:
                recursive_ros2_normalization(item)

    recursive_ros2_normalization(obj)

    # Type specific conversions

    if ros2_type_str == "visualization_msgs/msg/MarkerArray":
        for marker in obj.get("markers", []):
            if isinstance(marker, dict):
                if isinstance(marker.get("header"), dict):
                    marker["header"] = convert_dict_to_header(marker["header"])

                if isinstance(marker.get("lifetime"), dict):
                    marker["lifetime"] = convert_dict_to_duration(marker["lifetime"])

    if ros2_type_str == "sensor_msgs/msg/CameraInfo":
        for cap in ("D", "K", "R", "P"):
            if cap in obj:
                obj[cap.lower()] = obj.pop(cap)
