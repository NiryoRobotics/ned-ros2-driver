# /usr/bin/env python3

from typing import Dict, Any, Callable
import array


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
    # TODO(Thomas): Need to check if it is useful to call normalize_ROS1_type_to_ROS2() here

    for key, value in ros1_dict.items():
        # Ignore keys that are not in the ROS2 response
        if not hasattr(ros2_response, key):
            continue

        field = getattr(ros2_response, key)

        # Handle nested messages
        if hasattr(field, "__slots__") and isinstance(value, dict):
            nested_msg = type(field)()
            ros2_service_response_from_ros1_dict(nested_msg, value)
            setattr(ros2_response, key, nested_msg)

        # Handle list of nested messages
        elif isinstance(field, list) and isinstance(value, list):
            list_field = []
            for item in value:
                # Use first item in the list to determine the type
                if isinstance(item, dict) and field and hasattr(field[0], "__slots__"):
                    nested_item = type(field[0])()
                    ros2_service_response_from_ros1_dict(nested_item, item)
                    list_field.append(nested_item)
                else:
                    list_field.append(item)
            setattr(ros2_response, key, list_field)

        else:
            setattr(ros2_response, key, value)


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
