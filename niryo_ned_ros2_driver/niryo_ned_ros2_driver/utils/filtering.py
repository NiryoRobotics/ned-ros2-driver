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

import re
from typing import Dict, List
from .constants import BLACKLISTED_INTERFACES, INCOMPATIBLE_TYPES


def compile_regex_list(regex_list: List[str]) -> List[re.Pattern]:
    """
    Compiles a list of regex patterns into a list of compiled regex objects.

    This function takes a list of string patterns and converts each one into a compiled
    regular expression object. If any pattern is invalid, a ValueError is raised.

    Args:
        regex_list (List[str]): A list of string patterns to compile into regex objects.

    Returns:
        List[re.Pattern]: A list of compiled regex pattern objects.

    Raises:
        ValueError: If any regex pattern in the list is invalid.
    """
    compiled_patterns = []
    for pattern in regex_list:
        try:
            compiled_patterns.append(re.compile(pattern))
        except re.error as e:
            raise ValueError(f"Invalid regex pattern '{pattern}': {e}")
    return compiled_patterns


def matches_any(name: str, patterns: List[re.Pattern]) -> bool:
    """
    Checks if a string matches any of the provided regular expression patterns.

    Args:
        name (str): The string to check against patterns.
        patterns (List[re.Pattern]): A list of compiled regular expression patterns.

    Returns:
        bool: True if the string fully matches any of the patterns, False otherwise.
    """
    return any(re.fullmatch(pattern, name) for pattern in patterns)


def is_whitelisted(name: str, whitelist_regex_patterns: List[re.Pattern]) -> bool:
    """
    Check if a name matches any of the provided whitelist regex patterns.

    Args:
        name (str): The name to check against the whitelist patterns.
        whitelist_regex_patterns (List[re.Pattern]): A list of compiled regex patterns.

    Returns:
        bool: True if the name matches any of the whitelist patterns, False otherwise.
    """
    return matches_any(name, whitelist_regex_patterns)


BLACKLISTED_REGEX_PATTERNS = compile_regex_list(BLACKLISTED_INTERFACES)


def is_blacklisted(name: str) -> bool:
    """
    Check if the given name matches any of the blacklisted regex patterns.

    Args:
        name (str): The name to check against blacklisted patterns.

    Returns:
        bool: True if the name matches any blacklisted pattern, False otherwise.
    """
    return matches_any(name, BLACKLISTED_REGEX_PATTERNS)


def is_action_topic(topic: str) -> bool:
    """
    Check if a topic is an action topic.

    This function determines whether a given topic is related to ROS2 actions by checking for specific suffixes.

    Args: topic (str): The topic name to check

    Returns:
        bool: True if the topic contains an action suffix, False otherwise
    """
    action_suffixes = ["/goal", "/cancel", "/status", "/result", "/feedback"]
    return any(topic.endswith(suffix) for suffix in action_suffixes)


def is_non_existing_ros2_type(topic: str) -> bool:
    """
    Check if a topic is a non-existing ROS 2 type.

    This function determines if a given topic starts with any of the incompatible
    type prefixes defined in INCOMPATIBLE_TYPES.

    Args:
        topic (str): The topic name to check.

    Returns:
        bool: True if the topic is of a non-existing ROS 2 type, False otherwise.
    """
    return any(topic.startswith(incompatible) for incompatible in INCOMPATIBLE_TYPES)


def filter_topics(
    topic_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
    """
    Filter topics.

    This function filters a dictionary of topics and their types based on several conditions:
    1. The topic name must not be an action topic.
    2. The topic type must exist in ROS 2.
    3. The topic must not be blacklisted.
    4. The topic name must match at least one of the provided whitelist regex patterns.

    Args:
        topic_type_map (Dict[str, str]): Dictionary mapping topic names to their type names
        whitelist_regex_patterns (List[str]): List of regex patterns for whitelisting topics

    Returns:
        Dict[str, str]: Filtered dictionary containing only the topics that passed all filters
    """
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
    """
    Filter services.

    This function filters a dictionary of services and their types based on several conditions:
    1. The service name must match at least one of the provided whitelist regex patterns.
    2. The service type must exist in ROS 2.
    3. The service must not be blacklisted.

    Args:
        service_type_map (Dict[str, str]): A dictionary mapping service names to their types.
        whitelist_regex_patterns (List[str]): A list of regex patterns to whitelist services.

    Returns:
        Dict[str, str]: A filtered dictionary containing only the services that meet all criteria.
    """
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


def filter_actions(
    action_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
    """
    Filter actions.

    This function a dictionary of actions and their types based on several conditions:
    1. The action name must match at least one of the provided whitelist regex patterns.
    2. The action type must exist in ROS 2.
    3. The action must not be blacklisted.

    Args:
        action_type_map: Dictionary mapping action names to their corresponding ROS2 action types
        whitelist_regex_patterns: List of regex patterns for whitelisting actions

    Returns:
        A filtered dictionary containing only the actions that:
        1. Match at least one whitelist pattern
        2. Have valid existing ROS2 types
        3. Are not blacklisted
    """
    result = {}
    compiled_patterns = compile_regex_list(whitelist_regex_patterns)
    for action, action_type in action_type_map.items():
        if (
            not is_whitelisted(action, compiled_patterns)
            or is_non_existing_ros2_type(action_type)
            or is_blacklisted(action)
        ):
            continue
        result[action] = action_type
    return result
