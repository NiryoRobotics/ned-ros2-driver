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
        self._ttl = ttl_seconds
        self._checksum_cache: deque[Tuple[str, float]] = deque(maxlen=max_cache_size)

    def _compute_checksum(self, msg: Dict[str, Any]) -> str:
        """
        Compute a SHA256 checksum for a message dictionary.

        Args:
            msg (Dict[str, Any]): The message to hash.

        Returns:
            str: A hex digest string representing the checksum.
        """
        msg_str = json.dumps(msg, sort_keys=True, default=str)
        return hashlib.sha256(msg_str.encode("utf-8")).hexdigest()

    @property
    def ttl(self):
        return self._ttl

    @ttl.setter
    def ttl(self, value):
        self._ttl = value

    def should_forward(self, msg_to_hash: Dict[str, Any]) -> bool:
        """
        Check if a message should be forwarded.
        If allowed, the message's checksum is recorded with a timestamp.

        Returns:
            bool: True if the message should be forwarded, False if it should be filtered.
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
