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


def filter_actions(
    action_type_map: Dict[str, str], whitelist_regex_patterns: List[str]
) -> Dict[str, str]:
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
