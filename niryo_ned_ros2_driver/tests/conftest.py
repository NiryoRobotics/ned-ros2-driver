import pytest
from unittest.mock import MagicMock
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import roslibpy


@pytest.fixture
def mock_node():
    """Create a mock ROS2 node."""
    mock = MagicMock(spec=Node)
    mock.get_logger.return_value = MagicMock()
    mock.get_publishers_info_by_topic.return_value = []
    mock.get_subscriptions_info_by_topic.return_value = []
    return mock


@pytest.fixture
def mock_rosbridge():
    """Create a mock rosbridge client."""
    mock = MagicMock(spec=roslibpy.Ros)
    mock.is_connected = True
    return mock


@pytest.fixture
def mock_callback_group():
    """Create a mock callback group."""
    return MagicMock(spec=ReentrantCallbackGroup)
