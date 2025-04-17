# /usr/bin/env python3

from rclpy.node import Node

from rosidl_runtime_py.utilities import get_service
from rosidl_runtime_py import message_to_ordereddict

import roslibpy

from .models import ROSTypes
from .utils.conversion import ros2_service_response_from_ros1_dict


class Service:
    def __init__(
        self,
        node: Node,
        service_name: str,
        service_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
    ):
        self._node = node
        self._service_name = service_name
        self._service_types = service_types
        self._prefix = prefix
        self._rosbridge_client = rosbridge_client

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
        )

    def _ros2_callback(self, request, response):
        """
        Callback for the ROS2 service.
        """

        # Convert the ROS2 request to a ROS1 dict request
        request_dict = message_to_ordereddict(request)

        ros1_result = self._ros1_service_client.call(
            roslibpy.ServiceRequest(request_dict),
        )

        try:
            # Convert the ROS1 dict response to a ROS2 response message
            self._convert_ros1_response_to_ros2(ros1_result, response)
            ros2_service_response_from_ros1_dict(response, ros1_result)
        except AttributeError as e:
            self._node.get_logger().error(
                f"Failed to convert ROS1 → ROS2 service response for service {self._service_name}: {e}"
            )
            raise
