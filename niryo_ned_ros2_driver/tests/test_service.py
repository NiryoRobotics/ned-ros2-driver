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
from unittest.mock import Mock, MagicMock, patch
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import roslibpy
from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py.set_message import set_message_fields
from niryo_ned_ros2_driver.service import Service
from niryo_ned_ros2_driver.utils.models import ROSTypes

class TestService:
    """Test suite for the Service class."""

    @pytest.fixture
    def mock_node(self):
        """Create a mock ROS2 node."""
        node = Mock(spec=Node)
        node.get_logger.return_value = Mock()
        node.create_service = Mock()
        return node

    @pytest.fixture
    def mock_rosbridge_client(self):
        """Create a mock rosbridge client."""
        return Mock(spec=roslibpy.Ros)

    @pytest.fixture
    def mock_service_types(self):
        """Create mock service types."""
        return ROSTypes(
            ros1_type="std_srvs/SetBool",
            ros2_type="std_srvs/srv/SetBool"
        )

    @pytest.fixture
    def mock_callback_group(self):
        """Create a mock callback group."""
        return Mock(spec=ReentrantCallbackGroup)

    @pytest.fixture
    def service_instance(self, mock_node, mock_rosbridge_client, mock_service_types, mock_callback_group):
        """Create a Service instance for testing."""
        with patch('niryo_ned_ros2_driver.service.get_service') as mock_get_service:
            mock_srv_class = Mock()
            mock_srv_class.Response.get_fields_and_field_types.return_value = {'success': 'bool', 'message': 'string'}
            mock_get_service.return_value = mock_srv_class
            
            service = Service(
                node=mock_node,
                service_name="test_service",
                service_types=mock_service_types,
                prefix="ros2_",
                rosbridge_client=mock_rosbridge_client,
                callback_group=mock_callback_group
            )
            return service

    def test_init(self, mock_node, mock_rosbridge_client, mock_service_types, mock_callback_group):
        """Test Service initialization."""
        with patch('niryo_ned_ros2_driver.service.get_service') as mock_get_service:
            mock_srv_class = Mock()
            mock_srv_class.Response.get_fields_and_field_types.return_value = {'success': 'bool'}
            mock_get_service.return_value = mock_srv_class
            
            service = Service(
                node=mock_node,
                service_name="test_service",
                service_types=mock_service_types,
                prefix="ros2_",
                rosbridge_client=mock_rosbridge_client,
                callback_group=mock_callback_group
            )
            
            assert service._service_name == "test_service"
            assert service._prefix == "ros2_"
            assert service._node == mock_node
            mock_node.get_logger().debug.assert_called_once()
            mock_node.create_service.assert_called_once()

    def test_create_ros1_service_client(self, service_instance):
        """Test creation of ROS1 service client."""
        with patch('niryo_ned_ros2_driver.service.roslibpy.Service') as mock_service:
            client = service_instance._create_ros1_service_client()
            mock_service.assert_called_once_with(
                service_instance._rosbridge_client,
                service_instance._service_name,
                service_instance._service_types.ros1_type
            )

    def test_create_ros2_service_server(self, service_instance):
        """Test creation of ROS2 service server."""
        server = service_instance._create_ros2_service_server()
        service_instance._node.create_service.assert_called_with(
            service_instance._ros2_srv_class,
            f"{service_instance._prefix}{service_instance._service_name}",
            service_instance._ros2_callback,
            callback_group=service_instance._callback_group
        )

    @patch('niryo_ned_ros2_driver.service.ros2_message_to_dict')
    @patch('niryo_ned_ros2_driver.service.normalize_ROS1_type_to_ROS2')
    @patch('niryo_ned_ros2_driver.service.set_message_fields')
    def test_ros2_callback_success(self, mock_set_fields, mock_normalize, mock_to_dict, service_instance):
        """Test successful ROS2 callback execution."""
        # Setup mocks
        mock_request = Mock()
        mock_response = Mock()
        mock_to_dict.return_value = {'data': True}
        
        mock_ros1_result = {'success': True, 'message': 'OK'}
        service_instance._ros1_service_client.call = Mock(return_value=mock_ros1_result)
        
        # Execute callback
        result = service_instance._ros2_callback(mock_request, mock_response)
        
        # Verify calls
        mock_to_dict.assert_called_once_with(mock_request)
        service_instance._ros1_service_client.call.assert_called_once()
        mock_normalize.assert_called_once()
        mock_set_fields.assert_called_once_with(mock_response, mock_ros1_result)
        assert result == mock_response

    @patch('niryo_ned_ros2_driver.service.ros2_message_to_dict')
    @patch('niryo_ned_ros2_driver.service.set_message_fields')
    def test_ros2_callback_attribute_error(self, mock_set_fields, mock_to_dict, service_instance):
        """Test ROS2 callback handling AttributeError."""
        # Setup mocks
        mock_request = Mock()
        mock_response = Mock()
        mock_to_dict.return_value = {'data': True}
        
        mock_ros1_result = {'success': True, 'message': 'OK'}
        service_instance._ros1_service_client.call = Mock(return_value=mock_ros1_result)
        mock_set_fields.side_effect = AttributeError("Conversion failed")
        
        # Execute and verify exception
        with pytest.raises(AttributeError):
            service_instance._ros2_callback(mock_request, mock_response)
        
        service_instance._node.get_logger().error.assert_called_once()

    def test_service_name_with_prefix(self, service_instance):
        """Test that service is created with correct prefix."""
        expected_name = f"{service_instance._prefix}{service_instance._service_name}"
        service_instance._node.create_service.assert_called_with(
            service_instance._ros2_srv_class,
            expected_name,
            service_instance._ros2_callback,
            callback_group=service_instance._callback_group
        )

    def test_response_field_types_assignment(self, mock_node, mock_rosbridge_client, mock_service_types, mock_callback_group):
        """Test that response field types are correctly assigned."""
        with patch('niryo_ned_ros2_driver.service.get_service') as mock_get_service:
            expected_fields = {'success': 'bool', 'message': 'string'}
            mock_srv_class = Mock()
            mock_srv_class.Response.get_fields_and_field_types.return_value = expected_fields
            mock_get_service.return_value = mock_srv_class
            
            service = Service(
                node=mock_node,
                service_name="test_service",
                service_types=mock_service_types,
                prefix="ros2_",
                rosbridge_client=mock_rosbridge_client,
                callback_group=mock_callback_group
            )
            
            assert service._response_field_types == expected_fields
