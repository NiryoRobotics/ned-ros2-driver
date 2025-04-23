# /usr/bin/env python3

from typing import Dict, Any, Callable
import array

from rosidl_runtime_py.utilities import get_message

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


def ros2_service_response_from_ros1_dict(ros2_response: Any, ros1_dict: dict):
    """
    Recursively populate a ROS 2 service response instance from a ROS1 service response dictionary.

    Handles nested types and lists.
    """
    if not hasattr(ros2_response, "get_fields_and_field_types"):
        raise TypeError(f"{ros2_response} is not a ROS 2 message")

    field_types = ros2_response.get_fields_and_field_types()
    for field_name, ros1_value in ros1_dict.items():
        if not hasattr(ros2_response, field_name):
            continue

        field_type_str = field_types[field_name]

        # Handle list of nested messages
        if field_type_str.startswith("sequence<") and isinstance(ros1_value, list):
            inner_type = field_type_str[len("sequence<") : -1]

            # Handle primitive types
            if is_primitive_type(inner_type):
                setattr(ros2_response, field_name, ros1_value)
                continue

            msg_class = get_message(inner_type)

            ros2_list = []
            for item in ros1_value:
                if isinstance(item, dict):
                    nested_msg = msg_class()
                    ros2_service_response_from_ros1_dict(nested_msg, item)
                    ros2_list.append(nested_msg)
                else:
                    ros2_list.append(item)

            setattr(ros2_response, field_name, ros2_list)

        # # Handle nested messages
        elif isinstance(ros1_value, dict):
            msg_class = get_message(field_type_str)
            nested_msg = msg_class()
            ros2_service_response_from_ros1_dict(nested_msg, ros1_value)
            setattr(ros2_response, field_name, nested_msg)

        # Handle base types
        else:
            setattr(ros2_response, field_name, ros1_value)


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


def convert_ROS1_marker_array_to_ROS2(obj: Dict[str, Any]):
    for marker in obj.get("markers"):
        if isinstance(marker, dict):
            marker["header"] = convert_ROS1_header_to_ROS2(marker.get("header"))
            marker["lifetime"] = convert_ROS1_duration_to_ROS2(marker.get("lifetime"))


def convert_ROS1_camera_info_to_ROS2(obj: Dict[str, Any]):
    for cap in ("D", "K", "R", "P"):
        if cap in obj:
            obj[cap.lower()] = obj.pop(cap)


# Useful when we have access to the message type
ROS1_TO_ROS2_TYPE_CONVERSIONS: Dict[str, Callable] = {
    "visualization_msgs/msg/MarkerArray": convert_ROS1_marker_array_to_ROS2,
    "sensor_msgs/msg/CameraInfo": convert_ROS1_camera_info_to_ROS2,
}

# Useful when we only have access to the field name in a dictionary
ROS1_TO_ROS2_FIELD_CONVERSIONS: Dict[str, Callable] = {
    "header": convert_ROS1_header_to_ROS2,
    "time_from_start": convert_ROS1_duration_to_ROS2,
    "stamp": convert_ROS1_time_to_ROS2,
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


ROS2_TO_ROS1_FIELD_CONVERSIONS: Dict[str, Callable] = {
    "header": convert_ros2_header_to_ros1,
    "stamp": convert_ros2_time_to_ros1,
    "time_from_start": convert_ros2_duration_to_ros1,
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


def normalize_ROS2_type_to_ROS1(obj: dict):
    """
    Applies type-specific normalization of a ROS2 message dict to fit expected ROS1 format.
    """
    recursive_ros2_fields_to_ros1_normalization(obj)
