# /usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import qos_profile_default

import roslibpy

from .topic import Topic
from .models import ROSTypes
from .utils import convert_ros1_to_ros2_type


class ROS2Driver:
    def __init__(
        self,
        node: Node,
        namespace: str,
        ip: str,
        port: int,
        ros2_interfaces_package: str,
    ):
        self._node = node
        self._namespace = namespace
        self._ros2_interfaces_package = ros2_interfaces_package
        self._rosbridge_client = roslibpy.Ros(host=ip, port=port)

        self._topics = []
        self._services = []
        self._actions = []

        self._rosbridge_client.run()

        self._register_topics()

    def _register_topics(self):
        """
        Get all the topics available on the ROS1 side and create the corresponding
        topics on the ROS2 side.
        """
        topics = self._rosbridge_client.get_topics()
        for topic in topics:
            ros1_type = self._rosbridge_client.get_topic_type(topic)
            topic_type = ROSTypes(
                ros1_type=ros1_type,
                ros2_type=convert_ros1_to_ros2_type(
                    ros1_type, self._ros2_interfaces_package
                ),
            )
            # TODO handle special cases (Transient local , best effort, ...)
            qos = qos_profile_default
            self._topics.append(
                Topic(
                    self._node,
                    topic,
                    topic_type,
                    qos,
                    self._namespace,
                    self._rosbridge_client,
                )
            )
