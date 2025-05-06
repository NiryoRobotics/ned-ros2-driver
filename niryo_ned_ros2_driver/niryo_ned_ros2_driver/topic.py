# /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.set_message import set_message_fields

import roslibpy

from .models import ROSTypes
from .utils.conversion import (
    ros2_message_to_dict,
    normalize_ROS1_type_to_ROS2,
    normalize_ROS2_type_to_ROS1,
)
from .utils.constants import LATCHED_ROS1_TOPICS
from .utils.loopback_filter import LoopbackFilter


class Topic:
    def __init__(
        self,
        node: Node,
        topic_name: str,
        topic_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
        callback_group,
    ):
        self._node = node
        self._topic_name = topic_name
        self._topic_types = topic_types
        self._prefix = prefix
        self._qos = self._get_ros2_qos_for_topic(topic_name)
        self._rosbridge_client = rosbridge_client
        self._callback_group = callback_group

        self._previous_graph_info = None

        self._is_subscribed = False
        self._is_published = False

        self._node.get_logger().debug(
            f"Creating topic bridge for {topic_name} ({topic_types.ros1_type} → {topic_types.ros2_type})"
        )

        self._ros1_type_str = topic_types.ros1_type
        self._ros2_type_str = topic_types.ros2_type
        self._ros2_msg_class = get_message(self._ros2_type_str)

        self._loopback_filter = LoopbackFilter()

        self._ros1_publisher = self._create_ros1_publisher()
        self._ros2_publisher = self._create_ros2_publisher()

        self._ros2_subscriber = None
        self._ros1_subscriber = None

    @property
    def full_topic_name(self):
        return f"{self._prefix}{self._topic_name}"

    def update(self):
        """
        This function:
        - Check the subscriber count for the ROS2 topic. Once the topic is subscribed, instanciate the ROS1 subscriber.
        - Check the publisher count for the ROS2 topic. Once the topic is published, instanciate the ROS2 subscriber.
        """
        current_info = (
            self._node.get_publishers_info_by_topic(self.full_topic_name),
            self._node.get_subscriptions_info_by_topic(self.full_topic_name),
        )

        if current_info != self._previous_graph_info:
            self._previous_graph_info = current_info
            self._update_subscription()
            self._update_publication()

    def _update_subscription(self):
        """
        Check the subscriber count for the ROS2 topic.
        Once the topic is subscribed, instanciate the ROS1 subscriber.
        """
        num_subscribers = len(self._previous_graph_info[1])  # subscriptions info
        if num_subscribers >= 1 and not self._is_subscribed:
            self._ros1_subscriber = self._create_ros1_subscriber(self._ros1_publisher)
            self._is_subscribed = True
            self._node.get_logger().debug(
                f"[{self.full_topic_name}] Subscribed to ROS1 (ROS2 subscribers: {num_subscribers})"
            )
        elif num_subscribers == 0 and self._is_subscribed:
            self._ros1_publisher.unsubscribe()
            self._is_subscribed = False
            self._node.get_logger().debug(
                f"[{self.full_topic_name}] Unsubscribed from ROS1"
            )

    def _update_publication(self):
        """
        Check the publisher count for the ROS2 topic.
        Once the topic is published, instanciate the ROS2 subscriber.
        """
        num_publishers = len(self._previous_graph_info[0])  # publishers info
        if num_publishers > 1 and not self._is_published:
            self._ros2_subscriber = self._create_ros2_subscriber()
            self._is_published = True
            self._node.get_logger().debug(
                f"[{self.full_topic_name}] Subscribed to ROS2 (ROS2 publishers: {num_publishers})"
            )
        elif num_publishers <= 1 and self._is_published:
            self._node.destroy_subscription(self._ros2_subscriber)
            self._ros2_subscriber = None
            self._is_published = False
            self._node.get_logger().debug(
                f"[{self.full_topic_name}] Unsubscribed from ROS2"
            )

    def _create_ros2_publisher(self):
        """
        Create a ROS2 publisher for the topic.
        """
        return self._node.create_publisher(
            self._ros2_msg_class,
            self.full_topic_name,
            self._qos,
        )

    def _create_ros2_subscriber(self):
        """
        Create a ROS2 subscriber for the topic.
        """
        return self._node.create_subscription(
            self._ros2_msg_class,
            self.full_topic_name,
            self._ros2_callback,
            self._qos,
            callback_group=self._callback_group,
        )

    def _create_ros1_publisher(self):
        """
        Create a ROS1 publisher for the topic.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        )

    def _create_ros1_subscriber(self, topic: roslibpy.Topic):
        """
        Create a ROS1 subscriber for the topic.
        """
        return topic.subscribe(self._ros1_callback)

    def _ros1_callback(self, ros1_msg_dict):
        """
        Callback for the ROS1 subscriber.
        Converts the dictionary from roslibpy into a ROS2 message.
        """

        # Normalize the ROS1 message to match the expected ROS2 format
        normalize_ROS1_type_to_ROS2(ros1_msg_dict, self._ros2_type_str)

        # Check if the message hash is cached, cache it and forward it if not
        if not self._loopback_filter.should_forward(ros1_msg_dict):
            return

        try:
            ros2_msg = self._ros2_msg_class()
            set_message_fields(ros2_msg, ros1_msg_dict)
            if rclpy.ok():
                self._ros2_publisher.publish(ros2_msg)
        except Exception as e:
            self._node.get_logger().error(
                f"Failed to convert ROS1 → ROS2 message for topic '{self._topic_name}': {e}"
            )

    def _ros2_callback(self, ros2_msg):
        """
        Callback for the ROS2 subscriber.
        """
        try:
            msg_dict = ros2_message_to_dict(ros2_msg)

            # Check if the message hash is cached, cache it and forward it if not
            if not self._loopback_filter.should_forward(msg_dict):
                return

            # Normalize the ROS2 message to match the expected ROS1 format after the loopback check
            # This is important to ensure that the message format are similar for the hash comparison
            # in the loopback filter
            normalize_ROS2_type_to_ROS1(msg_dict, self._topic_types.ros1_type)
            self._ros1_publisher.publish(msg_dict)
        except Exception as e:
            self._node.get_logger().error(
                f"Failed to convert ROS2 → ROS1 message for topic '{self._topic_name}': {e}"
            )

    def _get_ros2_qos_for_topic(self, topic_name: str) -> QoSProfile:
        """
        Returns the appropriate ROS2 QoS profile for a given topic name.
        """
        if topic_name in LATCHED_ROS1_TOPICS:
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
        else:
            # Default to non-latched behavior
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
