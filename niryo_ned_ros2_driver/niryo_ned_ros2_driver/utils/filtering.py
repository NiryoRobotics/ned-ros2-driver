# /usr/bin/env python3

import re
from typing import Dict, List
from .constants import BLACKLISTED_INTERFACES


def compile_regex_list(regex_list: List[str]) -> List[re.Pattern]:
    compiled_patterns = []
    for pattern in regex_list:
        try:
            compiled_patterns.append(re.compile(pattern))
        except re.error as e:
            raise ValueError(f"Invalid regex pattern '{pattern}': {e}")
    return compiled_patterns


def matches_any(patterns, topic):
    return topic in patterns


def is_whitelisted(name: str, whitelist_regex_patterns: List[re.Pattern]) -> bool:
    """
    Check if the name matches any regex pattern in the whitelist.
    """
    return any(re.fullmatch(pattern, name) for pattern in whitelist_regex_patterns)


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


def filter_topics(
    topic_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
    result = {}
    compiled_patterns = compile_regex_list(whitelist_regex_patterns)
    for topic, topic_type in topic_type_map.items():
        if (
            is_action_topic(topic)
            or is_non_existing_ros2_topic_type(topic_type)
            or is_blacklisted_topic(topic)
            or not is_whitelisted(topic, compiled_patterns)
        ):
            continue
        result[topic] = topic_type
    return result
