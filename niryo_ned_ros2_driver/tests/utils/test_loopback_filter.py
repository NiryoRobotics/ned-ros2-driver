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

import time
from niryo_ned_ros2_driver.utils.loopback_filter import LoopbackFilter


class TestLoopbackFilter:
    """Test the LoopbackFilter utility."""

    def test_compute_checksum(self):
        """Test checksum computation for different messages."""
        filter = LoopbackFilter()

        msg1 = {"data": "test", "value": 123}
        msg2 = {"data": "test", "value": 456}
        msg3 = {"value": 123, "data": "test"}  # Same as msg1 but different order

        checksum1 = filter._compute_checksum(msg1)
        checksum2 = filter._compute_checksum(msg2)
        checksum3 = filter._compute_checksum(msg3)

        # Different messages should have different checksums
        assert checksum1 != checksum2

        # Same data in different order should have the same checksum
        assert checksum1 == checksum3

        # Test with nested structures
        msg4 = {"data": {"nested": "value"}, "array": [1, 2, 3]}
        msg5 = {"array": [1, 2, 3], "data": {"nested": "value"}}

        checksum4 = filter._compute_checksum(msg4)
        checksum5 = filter._compute_checksum(msg5)

        assert checksum4 == checksum5

    def test_should_forward(self):
        """Test duplicate detection functionality with should_forward method."""
        filter = LoopbackFilter(ttl_seconds=0.2)

        msg = {"data": "test", "value": 123}

        # First time should be forwarded
        assert filter.should_forward(msg)

        # Same message within TTL should not be forwarded
        assert not filter.should_forward(msg)

        # Different message should be forwarded
        msg2 = {"data": "different", "value": 456}
        assert filter.should_forward(msg2)

        # Test that TTL works - wait longer than TTL and it should be forwarded again
        time.sleep(0.3)  # Longer than the 0.2s TTL
        assert filter.should_forward(msg)

    def test_max_cache_size(self):
        """Test that the cache size limit works correctly."""
        filter = LoopbackFilter(max_cache_size=3)

        test_data = [
            {"data": "test0"},
            {"data": "test1"},
            {"data": "test2"},
            {"data": "test3"},
        ]

        # Add 4 different messages
        for data in test_data:
            filter.should_forward(data)

        # The cache should only contain the latest 3
        assert len(filter._checksum_cache) == 3

        # The others should still be in the cache and not be forwarded
        for i in range(1, 4):
            assert not filter.should_forward(test_data[i])

        # The oldest message should have been evicted and be allowed to forward again
        assert filter.should_forward(test_data[0])

    def test_header_extraction(self):
        """Test that only the header is used for filtering when available."""
        filter = LoopbackFilter()

        # Message with header
        msg1 = {
            "header": {"frame_id": "base_link", "stamp": {"sec": 10, "nsec": 0}},
            "data": "test_data",
        }

        # Same header but different data
        msg2 = {
            "header": {"frame_id": "base_link", "stamp": {"sec": 10, "nsec": 0}},
            "data": "different_data",
        }

        # Different header
        msg3 = {
            "header": {"frame_id": "base_link", "stamp": {"sec": 11, "nsec": 0}},
            "data": "test_data",
        }

        # First message should be forwarded
        assert filter.should_forward(msg1)

        # Second message with same header should not be forwarded
        assert not filter.should_forward(msg2)

        # Third message with different header should be forwarded
        assert filter.should_forward(msg3)
