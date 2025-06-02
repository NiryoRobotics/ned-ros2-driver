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
import base64

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
    """
    Check if a given type string represents a primitive type.

    Args:
        type_str (str): The string representation of the type to check.

    Returns:
        bool: True if the type is a primitive type, False otherwise.
    """
    return type_str in PRIMITIVE_TYPES


def ros2_message_to_dict(msg: Any) -> dict:
    """
    Convert a ROS2 message to a Python dictionary.

    This function recursively converts a ROS2 message and its nested attributes
    into a standard Python dictionary structure that is easier to manipulate
    and serialize.

    Args:
        msg (Any): The ROS2 message to convert. Can be a ROS2 message object,
                   a primitive type (int, float, bool, str), or a container type
                   (list, tuple, dict, array.array).

    Returns:
        dict: A dictionary representation of the ROS2 message. Complex message types
              are converted to nested dictionaries, while primitive types are preserved.
              Byte arrays are decoded to strings, and any unexpected types are converted
              to their string representation.
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
    Convert a ROS1 header dictionary to a ROS2 header dictionary format.

    This function takes a ROS1 header dictionary and transforms it into a ROS2 compatible header dictionary.

    Args:
        ros1_header (dict): The ROS1 header dictionary containing 'stamp' and optionally 'frame_id' keys.

    Returns:
        dict: A ROS2 compatible header dictionary with 'stamp' and 'frame_id' keys.
    """
    return {
        "stamp": convert_ROS1_time_to_ROS2(ros1_header.get("stamp")),
        "frame_id": ros1_header.get("frame_id", ""),
    }


def convert_ROS1_time_to_ROS2(ros1_time: dict) -> dict:
    """
    Convert ROS1 time format to ROS2 time format.

    ROS1 time uses "secs" and "nsecs" keys, while ROS2 uses "sec" and "nanosec".
    This function converts from the ROS1 format to the ROS2 format.

    Args:
        ros1_time (dict): A dictionary containing ROS1 time format with keys "secs" and "nsecs".

    Returns:
        dict: A dictionary containing ROS2 time format with keys "sec" and "nanosec".
    """
    return {
        "sec": ros1_time.get("secs", 0),
        "nanosec": ros1_time.get("nsecs", 0),
    }


def convert_ROS1_duration_to_ROS2(ros1_duration: dict) -> dict:
    """
    Convert a ROS1 duration dictionary to a ROS2 duration dictionary.

    ROS1 uses 'secs' and 'nsecs' keys while ROS2 uses 'sec' and 'nanosec' keys.
    This function converts from the ROS1 format to the ROS2 format.

    Args:
        ros1_duration (dict): A dictionary containing ROS1 time format with keys "secs" and "nsecs".

    Returns:
        dict: A dictionary containing ROS2 duration format with keys "sec" and "nanosec".
    """
    return {
        "sec": ros1_duration.get("secs", 0),
        "nanosec": ros1_duration.get("nsecs", 0),
    }


def convert_ROS1_compressed_image_to_ROS2(obj: Dict[str, Any]):
    """
    Convert a ROS1 CompressedImage message to ROS2 format.

    This function specifically handles the 'data' field in CompressedImage messages,
    ensuring it's properly converted from a string representation to a bytes array.

    Args:
        obj (Dict[str, Any]): A dictionary containing CompressedImage data in ROS1 format
    """
    if "data" in obj and isinstance(obj["data"], str):
        base64_bytes = obj["data"].encode("ascii")
        obj["data"] = base64.b64decode(base64_bytes)


def convert_ROS1_camera_info_to_ROS2(obj: Dict[str, Any]):
    """
    Convert ROS1 camera_info message format to ROS2 format.

    This function modifies the input dictionary by changing the capitalized matrix keys
    in ROS1 format ('D', 'K', 'R', 'P') to lowercase versions ('d', 'k', 'r', 'p')
    as required by ROS2.

    Args:
        obj (Dict[str, Any]): A dictionary containing camera_info data in ROS1 format
    """
    for cap in ("D", "K", "R", "P"):
        if cap in obj:
            obj[cap.lower()] = obj.pop(cap)


# Useful when we have access to the message type
ROS1_TO_ROS2_TYPE_CONVERSIONS: Dict[str, Callable] = {
    "sensor_msgs/msg/CameraInfo": convert_ROS1_camera_info_to_ROS2,
    "sensor_msgs/msg/CompressedImage": convert_ROS1_compressed_image_to_ROS2,
}

# Useful when we only have access to the field name in a dictionary
ROS1_TO_ROS2_FIELD_CONVERSIONS: Dict[str, Callable] = {
    "header": convert_ROS1_header_to_ROS2,
    "time_from_start": convert_ROS1_duration_to_ROS2,
    "stamp": convert_ROS1_time_to_ROS2,
    "lifetime": convert_ROS1_duration_to_ROS2,
}


def recursive_ros1_fields_to_ros2_normalization(obj: Any):
    """
    Recursively converts ROS1 field names to ROS2 field names in nested dictionaries and lists.

    This function traverses through dictionaries and lists, normalizing field names
    according to predefined conversion rules.

    Args:
        obj (Any): The object to normalize. Can be a dictionary, list, or other data type.
            If it's a dictionary, its keys will be checked against the conversion mapping.
            If it's a list, each item will be processed recursively.
            Other types remain unchanged.
    """
    if isinstance(obj, dict):
        for key, value in list(obj.items()):
            if key in ROS1_TO_ROS2_FIELD_CONVERSIONS and isinstance(value, dict):
                obj[key] = ROS1_TO_ROS2_FIELD_CONVERSIONS[key](value)
            else:
                recursive_ros1_fields_to_ros2_normalization(value)
    elif isinstance(obj, list):
        for item in obj:
            recursive_ros1_fields_to_ros2_normalization(item)


def normalize_ROS1_type_to_ROS2(obj: Dict[str, Any], ros2_type_str: str):
    """
    Normalize a ROS1 type object to comply with ROS2 format.

    This function first performs recursive field normalization between ROS1 and ROS2 formats,
    then applies any specific type conversion functions if the target ROS2 type is registered
    in the conversion dictionary.

    Args:
        obj (Dict[str, Any]): The ROS1 message dictionary to be normalized
        ros2_type_str (str): The target ROS2 message type string identifier
    """
    recursive_ros1_fields_to_ros2_normalization(obj)

    if ros2_type_str in ROS1_TO_ROS2_TYPE_CONVERSIONS:
        ROS1_TO_ROS2_TYPE_CONVERSIONS[ros2_type_str](obj)


# ---------------------------- ROS2 -> ROS1 ---------------------------- #


def convert_ros2_time_to_ros1(time: dict) -> dict:
    """
    Convert ROS 2 time dictionary to ROS 1 time dictionary format.

    Args:
        time (dict): A ROS 2 time dictionary with 'sec' and 'nanosec' keys.

    Returns:
        dict: A ROS 1 duration dictionary with 'secs' and 'nsecs' keys.
    """
    return {"secs": time.get("sec", 0), "nsecs": time.get("nanosec", 0)}


def convert_ros2_duration_to_ros1(duration: dict) -> dict:
    """
    Converts a ROS 2 duration dictionary to a ROS 1 duration dictionary format.

    Args:
        duration (dict): A ROS 2 duration dictionary with 'sec' and 'nanosec' keys.

    Returns:
        dict: A ROS 1 duration dictionary with 'secs' and 'nsecs' keys.
    """
    return {"secs": duration.get("sec", 0), "nsecs": duration.get("nanosec", 0)}


def convert_ros2_header_to_ros1(header: dict) -> dict:
    """
    Convert a ROS2 header dictionary to a ROS1 header dictionary.

    Args:
        header (dict): A ROS2 header dictionary containing 'stamp' (ROS2 time) and 'frame_id' fields.

    Returns:
        dict: A ROS1 header dictionary with converted 'stamp' field.
    """
    return {
        "stamp": convert_ros2_time_to_ros1(header.get("stamp")),
        "frame_id": header.get("frame_id", ""),
    }


def convert_ROS2_Follow_joint_traj_goal_to_ROS1(obj: Dict[str, Any]):
    """
    Converts a ROS2 FollowJointTrajectory goal object to ROS1 format by removing ROS2-specific fields.

    This function modifies the input object in-place by removing fields that don't exist in the ROS1
    FollowJointTrajectory action goal structure.

    Args:
        obj (Dict[str, Any]): Dictionary representing a ROS2 FollowJointTrajectory goal

    Note:
        Fields removed:
        - multi_dof_trajectory
        - component_path_tolerance
        - component_goal_tolerance
    """
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


def recursive_ros2_fields_to_ros1_normalization(obj: Any):
    """
    Recursively normalizes ROS2 fields to ROS1 format in nested data structures.

    This function traverses through dictionaries and lists, converting ROS2 field
    formats to their ROS1 equivalents using predefined conversion functions.

    Args:
        o (Any): The object to normalize. Can be a dictionary, list, or any other type.
                 If it's a dictionary or list, the function will recursively process its contents.
    """
    if isinstance(obj, dict):
        for key, value in list(obj.items()):
            if key in ROS2_TO_ROS1_FIELD_CONVERSIONS and isinstance(value, dict):
                obj[key] = ROS2_TO_ROS1_FIELD_CONVERSIONS[key](value)
            else:
                recursive_ros2_fields_to_ros1_normalization(value)
    elif isinstance(obj, list):
        for item in obj:
            recursive_ros2_fields_to_ros1_normalization(item)


def normalize_ROS2_type_to_ROS1(obj: dict, ros1_type_str: str):
    """
    Normalize a ROS2 type object to comply with ROS1 format.

    This function applies normalization to a ROS2 object to make it compatible with ROS1 type requirements.
    It first performs recursive field normalization and then applies specific type conversion if available.

    Args:
        obj (dict): The ROS2 object to normalize, represented as a dictionary
        ros1_type_str (str): String identifier for the target ROS1 type
    """

    recursive_ros2_fields_to_ros1_normalization(obj)

    if ros1_type_str in ROS2_TO_ROS1_TYPE_CONVERSIONS:
        ROS2_TO_ROS1_TYPE_CONVERSIONS[ros1_type_str](obj)
