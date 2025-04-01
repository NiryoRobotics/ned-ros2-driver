# /usr/bin/env python3

from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
from typing import Dict, Any


def convert_dict_to_time(d: Dict[str, Any]) -> Time:
    return Time(sec=d.get("secs", 0), nanosec=d.get("nsecs", 0))


def convert_dict_to_duration(d: Dict[str, Any]) -> Duration:
    return Duration(sec=d.get("secs", 0), nanosec=d.get("nsecs", 0))


def convert_dict_to_header(d: Dict[str, Any]) -> Header:
    d.pop("seq", None)
    return Header(
        stamp=convert_dict_to_time(d.get("stamp", {})), frame_id=d.get("frame_id", "")
    )


def normalize_ros1_type_to_ros2(obj: Dict[str, Any], ros2_type_str: str):
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
