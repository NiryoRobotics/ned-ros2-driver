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

from niryo_ned_ros2_driver.service import Service
from niryo_ned_ros2_driver.utils.models import ROSTypes


class TestService:
    """Test suite for the Service class that bridges ROS1 and ROS2 services."""

    @pytest.fixture
    def mock_get_service(self):
        """Create a mock for get_service function."""
        with patch("niryo_ned_ros2_driver.service.get_service") as mock:
            yield mock

    @pytest.fixture
    def mock_roslibpy_service_class(self):
        """Mock the roslibpy.Service class."""
        with patch("niryo_ned_ros2_driver.service.roslibpy.Service") as mock:
            yield mock

    @pytest.fixture
    def mock_ros1_service(self):
        """Create a mock ROS1 service."""
        mock = MagicMock(spec=roslibpy.Service)
        return mock

    @pytest.fixture
    def service_ros_types(self):
        """Create a ROSTypes fixture for service types."""
        return ROSTypes(ros1_type="std_srvs/Trigger", ros2_type="std_srvs/srv/Trigger")

    @pytest.fixture
    def service_instance(
        self,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        service_ros_types,
        mock_get_service,
        mock_roslibpy_service_class,
    ):
        """Create a Service instance for testing."""
        # Configure mocks
        mock_srv_class = MagicMock()
        mock_get_service.return_value = mock_srv_class

        mock_ros1_service = MagicMock()
        mock_roslibpy_service_class.return_value = mock_ros1_service

        # Create service instance
        service = Service(
            node=mock_node,
            service_name="/test_service",
            service_types=service_ros_types,
            prefix="",
            rosbridge_client=mock_rosbridge,
            callback_group=mock_callback_group,
        )

        return service

    def test_initialization(
        self,
        service_instance,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        service_ros_types,
        mock_get_service,
        mock_roslibpy_service_class,
    ):
        """Test correct initialization of the Service."""
        # Verify initialization
        assert service_instance._node == mock_node
        assert service_instance._service_name == "/test_service"
        assert service_instance._service_types == service_ros_types
        assert service_instance._prefix == ""
        assert service_instance._rosbridge_client == mock_rosbridge
        assert service_instance._callback_group == mock_callback_group

        # Verify service class was fetched
        mock_get_service.assert_called_once_with(service_ros_types.ros2_type)

        # Verify ROS1 service client was created
        mock_roslibpy_service_class.assert_called_once_with(
            mock_rosbridge, "/test_service", service_ros_types.ros1_type
        )

        # Verify ROS2 service server was created
        mock_node.create_service.assert_called_once_with(
            mock_get_service.return_value,
            "/test_service",
            service_instance._ros2_callback,
            callback_group=mock_callback_group,
        )

    def test_ros2_callback(self, service_instance):
        """Test ROS2 service callback that forwards to ROS1."""
        # Create mock request and response
        request = MagicMock()
        response = MagicMock()

        ros1_response = {"success": True, "message": "Operation successful"}

        # Mock the ROS1 service call
        service_instance._ros1_service_client.call.return_value = ros1_response

        with patch("niryo_ned_ros2_driver.service.ros2_message_to_dict"):
            with patch(
                "niryo_ned_ros2_driver.service.normalize_ROS1_type_to_ROS2"
            ) as mock_normalize:
                with patch(
                    "niryo_ned_ros2_driver.service.set_message_fields"
                ) as mock_set_fields:
                    result = service_instance._ros2_callback(request, response)

                    # Verify the complex response was normalized and set
                    mock_normalize.assert_called_once_with(
                        ros1_response, service_instance._service_types.ros2_type
                    )
                    mock_set_fields.assert_called_once_with(response, ros1_response)

                    # Verify the response was returned
                    assert result is response

    def test_ros2_callback_with_error(self, service_instance):
        """Test handling of errors in the ROS2 service callback."""
        # Create mock request and response
        request = MagicMock()
        response = MagicMock()

        # Mock an error in set_message_fields
        with patch("niryo_ned_ros2_driver.service.ros2_message_to_dict"):
            with patch("niryo_ned_ros2_driver.service.normalize_ROS1_type_to_ROS2"):
                with patch(
                    "niryo_ned_ros2_driver.service.set_message_fields"
                ) as mock_set_fields:
                    mock_set_fields.side_effect = AttributeError("No such attribute")

                    # Call the callback and expect an exception
                    with pytest.raises(AttributeError):
                        service_instance._ros2_callback(request, response)

                    # Verify error was logged
                    service_instance._node.get_logger().error.assert_called_once()

    def test_ros2_callback_with_complex_response(self, service_instance):
        """Test handling of complex responses with nested structures."""
        # Create mock request and response
        request = MagicMock()
        response = MagicMock()

        # Create a complex ROS1 response with nested structures
        complex_ros1_response = {
            "status": {"success": True, "message": "Operation completed"},
            "result": {
                "data": [1, 2, 3],
                "metadata": {"timestamp": {"secs": 10, "nsecs": 0}, "source": "robot"},
            },
        }

        # Mock the ROS1 service call to return the complex response
        service_instance._ros1_service_client.call.return_value = complex_ros1_response

        # Call the callback with mocked conversion functions
        with patch("niryo_ned_ros2_driver.service.ros2_message_to_dict"):
            with patch(
                "niryo_ned_ros2_driver.service.normalize_ROS1_type_to_ROS2"
            ) as mock_normalize:
                with patch(
                    "niryo_ned_ros2_driver.service.set_message_fields"
                ) as mock_set_fields:
                    service_instance._ros2_callback(request, response)

                    # Verify the complex response was normalized and set
                    mock_normalize.assert_called_once_with(
                        complex_ros1_response, service_instance._service_types.ros2_type
                    )
                    mock_set_fields.assert_called_once_with(
                        response, complex_ros1_response
                    )
