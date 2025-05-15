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

from rclpy.node import Node

from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py.set_message import set_message_fields

import roslibpy

from .utils.models import ROSTypes
from .utils.conversion import (
    ros2_message_to_dict,
    normalize_ROS1_type_to_ROS2,
)


class Service:
    def __init__(
        self,
        node: Node,
        service_name: str,
        service_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
        callback_group,
    ):
        self._node = node
        self._service_name = service_name
        self._service_types = service_types
        self._prefix = prefix
        self._rosbridge_client = rosbridge_client
        self._callback_group = callback_group

        self._node.get_logger().debug(
            f"Creating service bridge for {service_name} ({service_types.ros1_type} → {service_types.ros2_type})"
        )

        self._ros2_srv_class = get_service(service_types.ros2_type)

        self._ros2_service_server = self._create_ros2_service_server()
        self._ros1_service_client = self._create_ros1_service_client()

    def _create_ros1_service_client(self):
        """
        Create a ROS1 service client.
        """
        return roslibpy.Service(
            self._rosbridge_client,
            self._service_name,
            self._service_types.ros1_type,
        )

    def _create_ros2_service_server(self):
        """
        Create a ROS2 service server.
        """
        return self._node.create_service(
            self._ros2_srv_class,
            f"{self._prefix}{self._service_name}",
            self._ros2_callback,
            callback_group=self._callback_group,
        )

    def _ros2_callback(self, request, response):
        """
        Callback for the ROS2 service.
        """

        # Convert the ROS2 request to a ROS1 dict request
        request_dict = ros2_message_to_dict(request)

        ros1_result = self._ros1_service_client.call(
            roslibpy.ServiceRequest(request_dict),
        )

        try:
            normalize_ROS1_type_to_ROS2(ros1_result, self._service_types.ros2_type)
            # Convert the ROS1 dict response to a ROS2 response message
            set_message_fields(response, ros1_result)
        except AttributeError as e:
            self._node.get_logger().error(
                f"Failed to convert ROS1 → ROS2 service response for service {self._service_name}: {e}"
            )
            raise

        return response
