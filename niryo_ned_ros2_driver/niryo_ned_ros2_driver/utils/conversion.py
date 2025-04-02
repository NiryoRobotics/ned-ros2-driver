# /usr/bin/env python3

from typing import Dict, Any
from collections import OrderedDict


def normalize_ros1_type_to_ros2(obj: Dict[str, Any], ros2_type_str: str):
    def recursive_normalization(o):
        if isinstance(o, dict):
            for key, value in o.items():
                if key == "header" and isinstance(value, dict):
                    value.pop("seq", None)
                    o[key] = {
                        "stamp": {
                            "sec": value.get("stamp", {}).get("secs", 0),
                            "nanosec": value.get("stamp", {}).get("nsecs", 0),
                        },
                        "frame_id": value.get("frame_id", ""),
                    }
                elif key == "time_from_start" and isinstance(value, dict):
                    o[key] = {
                        "sec": value.get("secs", 0),
                        "nanosec": value.get("nsecs", 0),
                    }
                else:
                    recursive_normalization(value)
        elif isinstance(o, list):
            for item in o:
                recursive_normalization(item)

    recursive_normalization(obj)

    if ros2_type_str == "visualization_msgs/msg/MarkerArray":
        for marker in obj.get("markers", []):
            if isinstance(marker, dict):
                header = marker.get("header")
                if isinstance(header, dict):
                    header.pop("seq", None)
                    marker["header"] = {
                        "stamp": {
                            "sec": header.get("stamp", {}).get("secs", 0),
                            "nanosec": header.get("stamp", {}).get("nsecs", 0),
                        },
                        "frame_id": header.get("frame_id", ""),
                    }
                lifetime = marker.get("lifetime")
                if isinstance(lifetime, dict):
                    marker["lifetime"] = {
                        "sec": lifetime.get("secs", 0),
                        "nanosec": lifetime.get("nsecs", 0),
                    }

    if ros2_type_str == "sensor_msgs/msg/CameraInfo":
        for cap in ("D", "K", "R", "P"):
            if cap in obj:
                obj[cap.lower()] = obj.pop(cap)


def normalize_ros2_type_to_ros1(obj: dict):
    """
    Applies type-specific normalization of a ROS2 message dict to fit expected ROS1 format.
    """

    def convert_time(t):
        return {"secs": t.get("sec", 0), "nsecs": t.get("nanosec", 0)}

    def convert_header(h):
        if not isinstance(h, dict):
            return h
        return {
            "stamp": convert_time(h.get("stamp", {})),
            "frame_id": h.get("frame_id", ""),
        }

    def convert_duration(d):
        return {"secs": d.get("sec", 0), "nsecs": d.get("nanosec", 0)}

    def recursive_normalize(o):
        if isinstance(o, dict):
            for key, value in o.items():
                if key == "header" and isinstance(value, dict):
                    o[key] = convert_header(value)
                elif key in ("stamp", "time_from_start") and isinstance(value, dict):
                    o[key] = (
                        convert_duration(value)
                        if key != "stamp"
                        else convert_time(value)
                    )
                else:
                    recursive_normalize(value)
        elif isinstance(o, list):
            for item in o:
                recursive_normalize(item)

    recursive_normalize(obj)


def convert_ordereddict_to_dict(obj: Any) -> Any:
    if isinstance(obj, OrderedDict):
        return {k: convert_ordereddict_to_dict(v) for k, v in obj.items()}
    elif isinstance(obj, dict):
        return {k: convert_ordereddict_to_dict(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_ordereddict_to_dict(i) for i in obj]
    else:
        return obj
