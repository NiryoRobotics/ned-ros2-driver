# /usr/bin/env python3

import re
from typing import Dict, List
from .constants import BLACKLISTED_INTERFACES, INCOMPATIBLE_TYPES


def compile_regex_list(regex_list: List[str]) -> List[re.Pattern]:
    compiled_patterns = []
    for pattern in regex_list:
        try:
            compiled_patterns.append(re.compile(pattern))
        except re.error as e:
            raise ValueError(f"Invalid regex pattern '{pattern}': {e}")
    return compiled_patterns


def matches_any(name: str, patterns: List[re.Pattern]) -> bool:
    return any(re.fullmatch(pattern, name) for pattern in patterns)


def is_whitelisted(name: str, whitelist_regex_patterns: List[re.Pattern]) -> bool:
    """
    Check if the name matches any regex pattern in the whitelist.
    """
    return matches_any(name, whitelist_regex_patterns)


BLACKLISTED_REGEX_PATTERNS = compile_regex_list(BLACKLISTED_INTERFACES)


def is_blacklisted(name: str) -> bool:
    """
    Check if the name matches any regex pattern in the blacklist.
    """
    return matches_any(name, BLACKLISTED_REGEX_PATTERNS)


def is_action_topic(topic: str) -> bool:
    action_suffixes = ["/goal", "/cancel", "/status", "/result", "/feedback"]
    return any(topic.endswith(suffix) for suffix in action_suffixes)


def is_non_existing_ros2_type(topic: str) -> bool:
    return any(topic.startswith(incompatible) for incompatible in INCOMPATIBLE_TYPES)


def filter_topics(
    topic_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
    result = {}
    compiled_patterns = compile_regex_list(whitelist_regex_patterns)
    for topic, topic_type in topic_type_map.items():
        if (
            is_action_topic(topic)
            or is_non_existing_ros2_type(topic_type)
            or is_blacklisted(topic)
            or not is_whitelisted(topic, compiled_patterns)
        ):
            continue
        result[topic] = topic_type
    return result


def filter_services(
    service_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
    result = {}
    compiled_patterns = compile_regex_list(whitelist_regex_patterns)
    for service, service_type in service_type_map.items():
        if (
            not is_whitelisted(service, compiled_patterns)
            or is_non_existing_ros2_type(service_type)
            or is_blacklisted(service)
        ):
            continue
        result[service] = service_type
    return result
