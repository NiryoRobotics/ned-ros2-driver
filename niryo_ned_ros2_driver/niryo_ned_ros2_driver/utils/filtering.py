# /usr/bin/env python3

from typing import Dict
from .constants import BLACKLISTED_INTERFACES


def matches_any(patterns, topic):
    return topic in patterns


def is_blacklisted_topic(topic: str) -> bool:
    return matches_any(BLACKLISTED_INTERFACES, topic)


def is_action_topic(topic: str) -> bool:
    action_suffixes = ["/goal", "/cancel", "/status", "/result", "/feedback"]
    return any(topic.endswith(suffix) for suffix in action_suffixes)


def is_non_existing_ros2_topic_type(topic: str) -> bool:
    incompatible_topic_types = ["dynamic_reconfigure", "rosgraph_msgs", "bond"]
    return any(
        topic.startswith(incompatible) for incompatible in incompatible_topic_types
    )


def filter_topics(topic_type_map: Dict[str, str]) -> Dict[str, str]:
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
