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

import hashlib
import json
from collections import deque
from typing import Dict, Any, Tuple
import time

from rclpy.logging import get_logger

logger = get_logger("LoopbackFilter")


class LoopbackFilter:
    """
    A filter that prevents loopback message forwarding between ROS1 and ROS2.

    It uses checksums to identify recently forwarded messages and avoid
    sending them back to their origin, which would cause infinite loops.

    Each checksum is stored for a limited duration (TTL - Time To Live),
    allowing messages to be resent after a short delay.
    """

    def __init__(self, ttl_seconds: float = 0.1, max_cache_size: int = 50):
        """
        Initialize a loopback filter.

        A loopback filter is used to prevent processing duplicate messages within a short time period
        by caching checksums with timestamps.

        Args:
            ttl_seconds (float, optional): Time to live for cached items in seconds. Defaults to 0.1.
            max_cache_size (int, optional): Maximum number of checksums to store in the cache. Defaults to 50.
        """
        self._ttl = ttl_seconds
        self._checksum_cache: deque[Tuple[str, float]] = deque(maxlen=max_cache_size)

    def _compute_checksum(self, msg: Dict[str, Any]) -> str:
        """
        Computes a SHA-256 checksum for a given message.

        This method converts the message to a JSON string with sorted keys and
        computes a SHA-256 hash of the string's UTF-8 encoding.

        Args:
            msg (Dict[str, Any]): The message to compute a checksum for.
                Any non-serializable objects will be converted to strings.

        Returns:
            str: The hexadecimal representation of the SHA-256 hash.
        """
        msg_str = json.dumps(msg, sort_keys=True, default=str)
        return hashlib.sha256(msg_str.encode("utf-8")).hexdigest()

    @property
    def ttl(self):
        """
        Get the time-to-live value.

        Returns:
            float
                The time-to-live value in seconds.
        """
        return self._ttl

    @ttl.setter
    def ttl(self, value):
        """
        Setter for the Time-To-Live (TTL) attribute.

        Args
            value : float
                The new TTL value to set for the filter.

        Notes:
            This method sets the internal _ttl attribute which typically determines
            how long data should be considered valid in the loopback filter.
        """
        self._ttl = value

    def should_forward(self, msg_to_hash: Dict[str, Any]) -> bool:
        """
        Determines whether a message should be forwarded based on its content.

        This method checks if the message is a duplicate by computing a checksum of its content
        and comparing it against recently seen messages. Messages with the same checksum
        within the TTL period are considered duplicates and won't be forwarded.

        Args:
            msg_to_hash (Dict[str, Any]): The message to check, typically a dictionary that might contain a header.

        Returns:
            bool: True if the message should be forwarded (not a recent duplicate), False otherwise.

        Notes:
            - If the message contains a 'header' key, only the header is used for checksum calculation.
            - Expired entries (older than TTL) are automatically removed from the cache.
        """
        # Hash only the header part if available
        if msg_to_hash.get("header") is not None:
            msg_to_hash = msg_to_hash["header"]

        checksum = self._compute_checksum(msg_to_hash)
        now = time.monotonic()

        # Remove expired entries
        self._checksum_cache = deque(
            [(cs, ts) for cs, ts in self._checksum_cache if now - ts <= self._ttl],
            maxlen=self._checksum_cache.maxlen,
        )

        # Check for recent duplicate
        if any(cs == checksum for cs, ts in self._checksum_cache):
            return False

        self._checksum_cache.append((checksum, now))
        return True
