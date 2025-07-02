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
import roslibpy
from unittest.mock import MagicMock, patch
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from niryo_ned_ros2_driver.topic import Topic
from niryo_ned_ros2_driver.utils.models import ROSTypes


class TestTopic:
    """Test suite for the Topic class that bridges ROS1 and ROS2 topics."""

    @pytest.fixture
    def mock_roslibpy_topic_class(self):
        """Mock the roslibpy.Topic class."""
        with patch("niryo_ned_ros2_driver.topic.roslibpy.Topic") as mock:
            yield mock

    @pytest.fixture
    def mock_get_message(self):
        """Create a mock for get_message function."""
        with patch("niryo_ned_ros2_driver.topic.get_message") as mock:
            yield mock

    @pytest.fixture
    def basic_ros_types(self):
        """Create a basic ROSTypes fixture for standard message types."""
        return ROSTypes(ros1_type="std_msgs/String", ros2_type="std_msgs/msg/String")

    @pytest.fixture
    def mock_ros1_topic(self):
        """Create a mock ROS1 topic."""
        mock = MagicMock(spec=roslibpy.Topic)
        mock.subscribe.return_value = MagicMock()
        return mock

    @pytest.fixture
    def mock_set_message_fields(self):
        """Create a mock for set_message_fields function."""
        with patch("niryo_ned_ros2_driver.topic.set_message_fields") as mock:
            yield mock

    @pytest.fixture
    def mock_normalize_ros1_to_ros2(self):
        """Mock the normalize_ROS1_type_to_ROS2 function."""
        with patch("niryo_ned_ros2_driver.topic.normalize_ROS1_type_to_ROS2") as mock:
            yield mock

    @pytest.fixture
    def mock_normalize_ros2_to_ros1(self):
        """Mock the normalize_ROS2_type_to_ROS1 function."""
        with patch("niryo_ned_ros2_driver.topic.normalize_ROS2_type_to_ROS1") as mock:
            yield mock

    @pytest.fixture
    def mock_ros2_message_to_dict(self):
        """Mock the ros2_message_to_dict function."""
        with patch("niryo_ned_ros2_driver.topic.ros2_message_to_dict") as mock:
            mock.return_value = {"data": "test message"}
            yield mock

    @pytest.fixture
    def topic_instance(
        self,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        basic_ros_types,
        mock_get_message,
        mock_roslibpy_topic_class,
    ):
        """Create a Topic instance for testing."""
        # Configure mocks
        mock_msg_class = MagicMock()
        mock_get_message.return_value = mock_msg_class

        mock_ros1_topic = MagicMock()
        mock_roslibpy_topic_class.return_value = mock_ros1_topic

        # Create topic instance
        topic = Topic(
            node=mock_node,
            topic_name="/test_topic",
            topic_types=basic_ros_types,
            prefix="",
            rosbridge_client=mock_rosbridge,
            callback_group=mock_callback_group,
        )

        # Add some additional mocks for testing
        topic._ros2_publisher = MagicMock()

        return topic

    def test_initialization(
        self,
        topic_instance,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        basic_ros_types,
        mock_get_message,
        mock_roslibpy_topic_class,
    ):
        """Test correct initialization of the Topic."""
        # Verify initialization
        assert topic_instance._node == mock_node
        assert topic_instance._topic_name == "/test_topic"
        assert topic_instance._topic_types == basic_ros_types
        assert topic_instance._prefix == ""
        assert topic_instance._rosbridge_client == mock_rosbridge
        assert topic_instance._callback_group == mock_callback_group
        assert topic_instance._is_subscribed is False
        assert topic_instance._is_published is False
        assert topic_instance.full_topic_name == "/test_topic"

        # Verify message class was fetched
        mock_get_message.assert_called_once_with(basic_ros_types.ros2_type)

        # Verify ROS1 publisher was created
        mock_roslibpy_topic_class.assert_called_once_with(
            mock_rosbridge, "/test_topic", basic_ros_types.ros1_type
        )

        # Verify loopback filter was created
        assert hasattr(topic_instance, "_loopback_filter")

    def test_update_subscription_when_subscribers_added(
        self, topic_instance, mock_node
    ):
        """Test ROS1 subscriber creation when ROS2 subscribers are added."""
        # Mock subscribers being added
        mock_node.get_subscriptions_info_by_topic.return_value = [MagicMock()]
        mock_node.get_publishers_info_by_topic.return_value = []

        # Mock the creation function
        topic_instance._create_ros1_subscriber = MagicMock()

        # Update the topic
        topic_instance.update()

        # Verify the subscriber was created
        topic_instance._create_ros1_subscriber.assert_called_once_with(
            topic_instance._ros1_publisher
        )
        assert topic_instance._is_subscribed is True

    def test_update_subscription_when_subscribers_removed(
        self, topic_instance, mock_node
    ):
        """Test ROS1 subscriber removal when ROS2 subscribers are removed."""
        # Set initial state
        topic_instance._is_subscribed = True
        topic_instance._ros1_publisher = MagicMock()

        # Mock subscribers being removed
        mock_node.get_subscriptions_info_by_topic.return_value = []
        mock_node.get_publishers_info_by_topic.return_value = []

        # Update the topic
        topic_instance.update()

        # Verify the subscriber was unsubscribed
        topic_instance._ros1_publisher.unsubscribe.assert_called_once()
        assert topic_instance._is_subscribed is False

    def test_update_publication_when_publishers_added(self, topic_instance, mock_node):
        """Test ROS2 subscriber creation when ROS2 publishers are added."""
        # Mock publishers being added (more than 1, as the topic itself counts as 1)
        mock_node.get_publishers_info_by_topic.return_value = [MagicMock(), MagicMock()]
        mock_node.get_subscriptions_info_by_topic.return_value = []

        # Mock the creation function
        mock_subscriber = MagicMock()
        topic_instance._create_ros2_subscriber = MagicMock(return_value=mock_subscriber)

        # Update the topic
        topic_instance.update()

        # Verify the subscriber was created
        topic_instance._create_ros2_subscriber.assert_called_once()
        assert topic_instance._is_published is True
        assert topic_instance._ros2_subscriber == mock_subscriber

    def test_update_publication_when_publishers_removed(
        self, topic_instance, mock_node
    ):
        """Test ROS2 subscriber removal when ROS2 publishers are removed."""
        # Set initial state
        topic_instance._is_published = True
        topic_instance._ros2_subscriber = MagicMock()

        # Mock publishers being removed
        mock_node.get_publishers_info_by_topic.return_value = [
            MagicMock()
        ]  # Only one (the topic itself)
        mock_node.get_subscriptions_info_by_topic.return_value = []

        # Update the topic
        topic_instance.update()

        assert topic_instance._is_published is False
        assert topic_instance._ros2_subscriber is None

    def test_get_ros2_qos_for_latched_topic(self, topic_instance):
        """Test QoS selection for latched topics."""
        with patch("niryo_ned_ros2_driver.topic.LATCHED_ROS1_TOPICS", ["/test_topic"]):
            qos = topic_instance._get_ros2_qos_for_topic("/test_topic")

            assert qos.reliability == QoSReliabilityPolicy.RELIABLE
            assert qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
            assert qos.history == QoSHistoryPolicy.KEEP_LAST
            assert qos.depth == 1

    def test_get_ros2_qos_for_normal_topic(self, topic_instance):
        """Test QoS selection for normal (non-latched) topics."""
        with patch("niryo_ned_ros2_driver.topic.LATCHED_ROS1_TOPICS", []):
            qos = topic_instance._get_ros2_qos_for_topic("/test_topic")

            assert qos.reliability == QoSReliabilityPolicy.RELIABLE
            assert qos.durability == QoSDurabilityPolicy.VOLATILE
            assert qos.history == QoSHistoryPolicy.KEEP_LAST
            assert qos.depth == 10

    def test_ros1_callback(self, topic_instance, mock_set_message_fields):
        """Test ROS1 to ROS2 message conversion and publishing."""
        # Mock the loopback filter
        topic_instance._loopback_filter = MagicMock()
        topic_instance._loopback_filter.should_forward.return_value = True

        # Mock the message class
        mock_ros2_msg = MagicMock()
        topic_instance._ros2_msg_class.return_value = mock_ros2_msg

        topic_instance._message_field_types.return_type = MagicMock()

        # Create a test message
        ros1_msg = {"data": "test message"}

        # Call the callback
        with patch("niryo_ned_ros2_driver.topic.rclpy.ok", return_value=True):
            with patch(
                "niryo_ned_ros2_driver.topic.normalize_ROS1_type_to_ROS2"
            ) as mock_normalize:
                topic_instance._ros1_callback(ros1_msg)

                # Verify message was normalized, converted and published
                mock_normalize.assert_called_once_with(
                    ros1_msg, topic_instance._message_field_types
                )
                mock_set_message_fields.assert_called_once_with(mock_ros2_msg, ros1_msg)
                topic_instance._ros2_publisher.publish.assert_called_once_with(
                    mock_ros2_msg
                )

    def test_ros1_callback_filtered(self, topic_instance, mock_set_message_fields):
        """Test that duplicate ROS1 messages are filtered out."""
        # Mock the loopback filter to reject the message
        topic_instance._loopback_filter = MagicMock()
        topic_instance._loopback_filter.should_forward.return_value = False

        # Create a test message
        ros1_msg = {"data": "test message"}

        # Call the callback
        topic_instance._ros1_callback(ros1_msg)

        # Verify the message was filtered and not published
        mock_set_message_fields.assert_not_called()
        topic_instance._ros2_publisher.publish.assert_not_called()

    def test_ros2_callback(self, topic_instance, mock_ros2_message_to_dict):
        """Test ROS2 to ROS1 message conversion and publishing."""
        # Mock the loopback filter
        topic_instance._loopback_filter = MagicMock()
        topic_instance._loopback_filter.should_forward.return_value = True

        # Create a test message
        ros2_msg = MagicMock()

        # Call the callback
        with patch(
            "niryo_ned_ros2_driver.topic.normalize_ROS2_type_to_ROS1"
        ) as mock_normalize:
            topic_instance._ros2_callback(ros2_msg)

            # Verify message was converted, normalized and published
            mock_ros2_message_to_dict.assert_called_once_with(ros2_msg)
            mock_normalize.assert_called_once_with({"data": "test message"})
            topic_instance._ros1_publisher.publish.assert_called_once_with(
                {"data": "test message"}
            )

    def test_ros2_callback_filtered(self, topic_instance, mock_ros2_message_to_dict):
        """Test that duplicate ROS2 messages are filtered out."""
        # Mock the loopback filter to reject the message
        topic_instance._loopback_filter = MagicMock()
        topic_instance._loopback_filter.should_forward.return_value = False

        # Create a test message
        ros2_msg = MagicMock()

        # Call the callback
        topic_instance._ros2_callback(ros2_msg)

        # Verify the message was filtered and not published
        mock_ros2_message_to_dict.assert_called_once()
        topic_instance._ros1_publisher.publish.assert_not_called()

    def test_exception_handling_in_ros1_callback(
        self, topic_instance, mock_set_message_fields
    ):
        """Test exception handling in ROS1 callback."""
        # Mock an exception during message conversion
        mock_set_message_fields.side_effect = Exception("Failed to convert")

        # Create a test message
        ros1_msg = {"data": "test message"}

        # Call the callback
        with patch("niryo_ned_ros2_driver.topic.normalize_ROS1_type_to_ROS2"):
            topic_instance._ros1_callback(ros1_msg)

            # Verify error was logged but no exception was raised
            topic_instance._node.get_logger().error.assert_called_once()

    def test_exception_handling_in_ros2_callback(
        self, topic_instance, mock_ros2_message_to_dict
    ):
        """Test exception handling in ROS2 callback."""
        # Mock an exception during message conversion
        mock_ros2_message_to_dict.side_effect = Exception("Failed to convert")

        # Create a test message
        ros2_msg = MagicMock()

        # Call the callback
        topic_instance._ros2_callback(ros2_msg)

        # Verify error was logged but no exception was raised
        topic_instance._node.get_logger().error.assert_called_once()
