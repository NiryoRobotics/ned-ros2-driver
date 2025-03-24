# /usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile

from rosidl_runtime_py.utilities import get_message

import roslibpy

from .models import ROSTypes


class Topic:
    def __init__(
        self,
        node: Node,
        topic_name: str,
        topic_types: ROSTypes,
        qos: QoSProfile,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
    ):
        self._node = node
        self._topic_name = topic_name
        self._topic_types = topic_types
        self._prefix = prefix
        self._qos = qos
        self._rosbridge_client = rosbridge_client

        # TODO reprendre ici -> création des messages ROS2 (mapping)
        # Générer du code pour maintenir la persistence

        # TODO différencier les messages custom Niryo des messages standards de ROS
        node.get_logger().info(f"Topic type {self._topic_types.ros2_type}")
        self._ros2_type_obj = get_message(self._topic_types.ros2_type)
        message_details = self._rosbridge_client.get_message_details(
            self._topic_types.ros1_type
        )
        node.get_logger().info(f"Message details {message_details}")

        self._ros2_subscriber = self._create_ros2_subscriber()
        self._ros2_publisher = self._create_ros2_publisher()
        self._ros1_subscriber = self._create_ros1_subscriber()
        self._ros1_publisher = self._create_ros1_publisher()

    def _generate_ros2_msg(self):
        """
        Generate a ROS2 message from the ROS1 message.
        """

    def _create_ros2_publisher(self):
        """
        Create a ROS2 publisher for the topic.
        """
        return self._node.create_publisher(
            self._ros2_type_obj,
            f"{self._prefix}{self._topic_name}",
            self._qos,
        )

    def _create_ros2_subscriber(self):
        """
        Create a ROS2 subscriber for the topic.
        """
        return self._node.create_subscription(
            self._ros2_type_obj,
            f"{self._prefix}{self._topic_name}",
            self._ros2_callback,
            self._qos,
        )

    def _create_ros1_publisher(self):
        """
        Create a ROS1 publisher for the topic.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        )

    def _create_ros1_subscriber(self):
        """
        Create a ROS1 subscriber for the topic.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        ).subscribe(self._ros1_callback)

    def _ros1_callback(self, ros1_msg):
        """
        Callback for the ROS1 subscriber.
        """
        # TODO convert ros1 msg to ros2
        # self._ros2_publisher.publish(msg)

    def _ros2_callback(self, msg):
        """
        Callback for the ROS2 subscriber.
        """
        # TODO convert ros2 msg to ros1
        # self._ros1_publisher.publish(msg)
