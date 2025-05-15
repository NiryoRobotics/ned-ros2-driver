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

import pytest
from unittest.mock import MagicMock, patch
import hashlib

from niryo_ned_ros2_driver.tf_static_topic import StaticTFTopic
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as Ros2Time
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


class TestStaticTFTopic:
    """Test suite for the StaticTFTopic class."""

    @pytest.fixture
    def ros_transform_dict(self):
        """Create a sample ROS transform dictionary."""
        return {
            "header": {
                "stamp": {"secs": 10, "nsecs": 500000000},
                "frame_id": "parent_frame",
                "seq": 10,
            },
            "child_frame_id": "child_frame",
            "transform": {
                "translation": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

    @pytest.fixture
    def ros_time_dict(self):
        """Create a sample ROS time dictionary."""
        return {"secs": 10, "nsecs": 500000000}

    @pytest.fixture
    def tf_ros_types(self):
        """Create a ROSTypes fixture for TF message types."""
        from niryo_ned_ros2_driver.utils.models import ROSTypes

        return ROSTypes(
            ros1_type="tf2_msgs/TFMessage", ros2_type="tf2_msgs/msg/TFMessage"
        )

    @pytest.fixture
    def tf_topic_instance(
        self, mock_node, mock_rosbridge, mock_callback_group, tf_ros_types
    ):
        """Create a StaticTFTopic instance for testing."""
        with patch("niryo_ned_ros2_driver.topic.get_message", return_value=TFMessage):
            with patch(
                "niryo_ned_ros2_driver.topic.roslibpy.Topic", return_value=MagicMock()
            ):
                with patch("niryo_ned_ros2_driver.topic.Topic.__init__") as mock_init:
                    mock_init.return_value = None

                    topic = StaticTFTopic(
                        node=mock_node,
                        topic_name="/tf_static",
                        topic_types=tf_ros_types,
                        prefix="",
                        rosbridge_client=mock_rosbridge,
                        callback_group=mock_callback_group,
                    )

                    # Set up necessary attributes
                    topic._ros2_publisher = MagicMock()

                    return topic

    def test_initialization(self, tf_topic_instance):
        """Test that the StaticTFTopic is initialized correctly."""
        assert hasattr(tf_topic_instance, "_published_hashes")
        assert isinstance(tf_topic_instance._published_hashes, dict)

    def test_get_ros2_qos_for_topic(self, tf_topic_instance):
        """Test that the correct QoS profile is returned for static TF topics."""
        qos = tf_topic_instance._get_ros2_qos_for_topic("/tf_static")

        assert qos.reliability == QoSReliabilityPolicy.RELIABLE
        assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
        assert qos.history == QoSHistoryPolicy.KEEP_ALL

    def test_hash_transform(self, tf_topic_instance, ros_transform_dict):
        """Test the transform hashing function."""
        # Calculate expected hash
        expected_s = "parent_frame_child_frame_1.0_2.0_3.0_0.0_0.0_0.0_1.0"
        expected_hash = hashlib.md5(expected_s.encode()).hexdigest()

        # Calculate actual hash
        actual_hash = tf_topic_instance._hash_transform(ros_transform_dict)

        # Verify they match
        assert actual_hash == expected_hash

        # Verify the hash changes if the transform changes
        ros_transform_dict["transform"]["translation"]["x"] = 1.1
        assert tf_topic_instance._hash_transform(ros_transform_dict) != expected_hash

    def test_ros1_callback_with_empty_message(self, tf_topic_instance):
        """Test callback with an empty message."""
        msg_dict = {}
        tf_topic_instance._ros1_callback(msg_dict)
        tf_topic_instance._ros2_publisher.publish.assert_not_called()

        # Test with empty transforms list
        msg_dict = {"transforms": []}
        tf_topic_instance._ros1_callback(msg_dict)
        tf_topic_instance._ros2_publisher.publish.assert_not_called()

    def test_convert_time(self, tf_topic_instance, ros_time_dict):
        """Test the time conversion function."""
        ros2_time = tf_topic_instance._convert_time(ros_time_dict)
        assert isinstance(ros2_time, Ros2Time)
        assert ros2_time.sec == 10
        assert ros2_time.nanosec == 500000000

    def test_convert_to_ros2_transform(self, tf_topic_instance, ros_transform_dict):
        """Test the transform conversion function."""
        # Convert to ROS2
        ros2_transform = tf_topic_instance._convert_to_ros2_transform(
            ros_transform_dict, "parent_frame", "child_frame"
        )

        # Verify the conversion
        assert isinstance(ros2_transform, TransformStamped)
        assert ros2_transform.header.stamp.sec == 10
        assert ros2_transform.header.stamp.nanosec == 500000000
        assert ros2_transform.header.frame_id == "parent_frame"
        assert ros2_transform.child_frame_id == "child_frame"
        assert ros2_transform.transform.translation.x == 1.0
        assert ros2_transform.transform.translation.y == 2.0
        assert ros2_transform.transform.translation.z == 3.0
        assert ros2_transform.transform.rotation.x == 0.0
        assert ros2_transform.transform.rotation.y == 0.0
        assert ros2_transform.transform.rotation.z == 0.0
        assert ros2_transform.transform.rotation.w == 1.0

    def test_ros1_callback_with_new_transform(
        self, tf_topic_instance, ros_transform_dict
    ):
        """Test callback with a new transform that hasn't been seen before."""
        # Create a test message with a transform
        msg_dict = {"transforms": [ros_transform_dict]}

        # Mock the conversion function
        ros2_transform = TransformStamped()
        tf_topic_instance._convert_to_ros2_transform = MagicMock(
            return_value=ros2_transform
        )

        # Call the callback
        tf_topic_instance._ros1_callback(msg_dict)

        # Verify the transform was converted and published
        tf_topic_instance._convert_to_ros2_transform.assert_called_once()
        tf_topic_instance._ros2_publisher.publish.assert_called_once()

        # Verify that the hash was stored
        assert ("parent_frame", "child_frame") in tf_topic_instance._published_hashes

    def test_ros1_callback_with_duplicate_transform(
        self, tf_topic_instance, ros_transform_dict
    ):
        """Test callback with a transform that has been seen before."""
        # Calculate the hash
        transform_hash = tf_topic_instance._hash_transform(ros_transform_dict)

        # Store the hash as if we'd seen this transform before
        tf_topic_instance._published_hashes[("parent_frame", "child_frame")] = (
            transform_hash
        )

        # Create a message with the same transform
        msg_dict = {"transforms": [ros_transform_dict]}

        # Mock the conversion function
        tf_topic_instance._convert_to_ros2_transform = MagicMock()

        # Call the callback
        tf_topic_instance._ros1_callback(msg_dict)

        # Verify that the transform was not converted or published
        tf_topic_instance._convert_to_ros2_transform.assert_not_called()
        tf_topic_instance._ros2_publisher.publish.assert_not_called()

    def test_ros1_callback_with_updated_transform(
        self, tf_topic_instance, ros_transform_dict
    ):
        """Test callback with a transform that has changed since last time."""
        # Store an old hash
        old_hash = "old_hash_value"
        tf_topic_instance._published_hashes[("parent_frame", "child_frame")] = old_hash

        # Create a message with a new transform that will have a different hash
        msg_dict = {"transforms": [ros_transform_dict]}

        # Mock the conversion function
        ros2_transform = TransformStamped()
        tf_topic_instance._convert_to_ros2_transform = MagicMock(
            return_value=ros2_transform
        )

        # Mock hash_transform to return a different hash
        new_hash = "new_hash_value"
        with patch.object(tf_topic_instance, "_hash_transform", return_value=new_hash):
            # Call the callback
            tf_topic_instance._ros1_callback(msg_dict)

            # Verify the transform was converted and published since hash changed
            tf_topic_instance._convert_to_ros2_transform.assert_called_once()
            tf_topic_instance._ros2_publisher.publish.assert_called_once()

            # Verify that the hash was updated
            assert (
                tf_topic_instance._published_hashes[("parent_frame", "child_frame")]
                == new_hash
            )

    def test_ros1_callback_with_multiple_transforms(self, tf_topic_instance):
        """Test callback with multiple transforms, some new, some duplicate."""
        # Create two transform dictionaries
        transform1 = {
            "header": {"frame_id": "/parent1", "stamp": {"secs": 10, "nsecs": 0}},
            "child_frame_id": "/child1",
            "transform": {
                "translation": {"x": 1.0, "y": 2.0, "z": 3.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

        transform2 = {
            "header": {"frame_id": "/parent2", "stamp": {"secs": 10, "nsecs": 0}},
            "child_frame_id": "/child2",
            "transform": {
                "translation": {"x": 4.0, "y": 5.0, "z": 6.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }

        # Store hash for transform1 (so it's a duplicate)
        hash1 = tf_topic_instance._hash_transform(transform1)
        tf_topic_instance._published_hashes[("parent1", "child1")] = hash1

        # Create a message with both transforms
        msg_dict = {"transforms": [transform1, transform2]}

        # Mock the conversion function
        ros2_transform = TransformStamped()
        tf_topic_instance._convert_to_ros2_transform = MagicMock(
            return_value=ros2_transform
        )

        # Call the callback
        tf_topic_instance._ros1_callback(msg_dict)

        # Verify the second transform was converted (not the first, which was duplicate)
        assert tf_topic_instance._convert_to_ros2_transform.call_count == 1
        tf_topic_instance._convert_to_ros2_transform.assert_called_with(
            transform2, "parent2", "child2"
        )

        # Verify a message was published with the transforms
        tf_topic_instance._ros2_publisher.publish.assert_called_once()

        # Verify both hashes are now stored
        assert ("parent1", "child1") in tf_topic_instance._published_hashes
        assert ("parent2", "child2") in tf_topic_instance._published_hashes

    def test_ros2_callback(self, tf_topic_instance):
        """Test that the ROS2 callback is empty (static TFs are not published from ROS2 to ROS1)."""
        # Create a mock ROS2 message
        msg = MagicMock()

        # Call the callback and verify nothing happens
        result = tf_topic_instance._ros2_callback(msg)
        assert result is None
