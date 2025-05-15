# Copyright (c) 2025 Niryo.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# /usr/bin/env python3

from typing import Dict, Any, Callable
import array

PRIMITIVE_TYPES = {
    "bool",
    "byte",
    "char",
    "float",
    "float32",
    "float64",
    "double",
    "short",
    "int",
    "long",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "string",
    "wstring",
}


def is_primitive_type(type_str: str) -> bool:
    return type_str in PRIMITIVE_TYPES


def ros2_message_to_dict(msg: Any) -> dict:
    """
    Recursively convert a ROS 2 message into a plain Python dict.

    Handles nested messages and sequences.

    :param msg: A ROS 2 message instance.
    :return: A plain Python dictionary.
    """
    if hasattr(msg, "get_fields_and_field_types") and hasattr(msg, "SLOT_TYPES"):
        result = {}
        for field_name in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field_name)
            result[field_name] = ros2_message_to_dict(value)
        return result
    elif isinstance(msg, array.array):
        return list(msg)
    elif isinstance(msg, (list, tuple)):
        return [ros2_message_to_dict(item) for item in msg]
    elif isinstance(msg, (bytes, bytearray)):
        return msg.decode(errors="ignore")
    elif isinstance(msg, (int, float, bool, str)):
        return msg
    elif isinstance(msg, dict):
        return {k: ros2_message_to_dict(v) for k, v in msg.items()}
    else:
        return str(msg)  # fallback for any unexpected types


# ---------------------------- Conversion functions ---------------------------- #

# ---------------------------- ROS1 -> ROS2 ---------------------------- #


def convert_ROS1_header_to_ROS2(ros1_header: dict) -> dict:
    """
    Convert ROS1 header format to ROS2 header format.
    """
    return {
        "stamp": convert_ROS1_time_to_ROS2(ros1_header.get("stamp")),
        "frame_id": ros1_header.get("frame_id", ""),
    }


def convert_ROS1_time_to_ROS2(ros1_time: dict) -> dict:
    """
    Convert ROS1 time format to ROS2 time format.
    """
    return {
        "sec": ros1_time.get("secs", 0),
        "nanosec": ros1_time.get("nsecs", 0),
    }


def convert_ROS1_duration_to_ROS2(ros1_duration: dict) -> dict:
    """
    Convert ROS1 duration format to ROS2 duration format.
    """
    return {
        "sec": ros1_duration.get("secs", 0),
        "nanosec": ros1_duration.get("nsecs", 0),
    }


def convert_ROS1_camera_info_to_ROS2(obj: Dict[str, Any]):
    for cap in ("D", "K", "R", "P"):
        if cap in obj:
            obj[cap.lower()] = obj.pop(cap)


# Useful when we have access to the message type
ROS1_TO_ROS2_TYPE_CONVERSIONS: Dict[str, Callable] = {
    "sensor_msgs/msg/CameraInfo": convert_ROS1_camera_info_to_ROS2,
}

# Useful when we only have access to the field name in a dictionary
ROS1_TO_ROS2_FIELD_CONVERSIONS: Dict[str, Callable] = {
    "header": convert_ROS1_header_to_ROS2,
    "time_from_start": convert_ROS1_duration_to_ROS2,
    "stamp": convert_ROS1_time_to_ROS2,
    "lifetime": convert_ROS1_duration_to_ROS2,
}


def recursive_ros1_fields_to_ros2_normalization(o: Any):
    if isinstance(o, dict):
        for key, value in list(o.items()):
            if key in ROS1_TO_ROS2_FIELD_CONVERSIONS and isinstance(value, dict):
                o[key] = ROS1_TO_ROS2_FIELD_CONVERSIONS[key](value)
            else:
                recursive_ros1_fields_to_ros2_normalization(value)
    elif isinstance(o, list):
        for item in o:
            recursive_ros1_fields_to_ros2_normalization(item)


def normalize_ROS1_type_to_ROS2(obj: Dict[str, Any], ros2_type_str: str):
    """
    Applies type-specific normalization of a ROS1 message dict to fit expected ROS2 format.
    """
    recursive_ros1_fields_to_ros2_normalization(obj)

    if ros2_type_str in ROS1_TO_ROS2_TYPE_CONVERSIONS:
        ROS1_TO_ROS2_TYPE_CONVERSIONS[ros2_type_str](obj)


# ---------------------------- ROS2 -> ROS1 ---------------------------- #


def convert_ros2_time_to_ros1(time: dict) -> dict:
    """
    Convert ROS2 time format to ROS1 time format.
    """
    return {"secs": time.get("sec", 0), "nsecs": time.get("nanosec", 0)}


def convert_ros2_duration_to_ros1(duration: dict) -> dict:
    """
    Convert ROS2 duration format to ROS1 duration format.
    """
    return {"secs": duration.get("sec", 0), "nsecs": duration.get("nanosec", 0)}


def convert_ros2_header_to_ros1(header: dict) -> dict:
    """
    Convert ROS2 header format to ROS1 header format.
    """
    return {
        "stamp": convert_ros2_time_to_ros1(header.get("stamp")),
        "frame_id": header.get("frame_id", ""),
    }


def convert_ROS2_Follow_joint_traj_goal_to_ROS1(obj: Dict[str, Any]):
    obj.pop("multi_dof_trajectory", None)
    obj.pop("component_path_tolerance", None)
    obj.pop("component_goal_tolerance", None)


# Useful when we have access to the message type
ROS2_TO_ROS1_TYPE_CONVERSIONS: Dict[str, Callable] = {
    "control_msgs/FollowJointTrajectoryAction": convert_ROS2_Follow_joint_traj_goal_to_ROS1,
}


ROS2_TO_ROS1_FIELD_CONVERSIONS: Dict[str, Callable] = {
    "header": convert_ros2_header_to_ros1,
    "stamp": convert_ros2_time_to_ros1,
    "time_from_start": convert_ros2_duration_to_ros1,
    "goal_time_tolerance": convert_ros2_duration_to_ros1,
}


def recursive_ros2_fields_to_ros1_normalization(o: Any):
    if isinstance(o, dict):
        for key, value in list(o.items()):
            if key in ROS2_TO_ROS1_FIELD_CONVERSIONS and isinstance(value, dict):
                o[key] = ROS2_TO_ROS1_FIELD_CONVERSIONS[key](value)
            else:
                recursive_ros2_fields_to_ros1_normalization(value)
    elif isinstance(o, list):
        for item in o:
            recursive_ros2_fields_to_ros1_normalization(item)


def normalize_ROS2_type_to_ROS1(obj: dict, ros1_type_str: str):
    """
    Applies type-specific normalization of a ROS2 message dict to fit expected ROS1 format.
    """
    recursive_ros2_fields_to_ros1_normalization(obj)

    if ros1_type_str in ROS2_TO_ROS1_TYPE_CONVERSIONS:
        ROS2_TO_ROS1_TYPE_CONVERSIONS[ros1_type_str](obj)
