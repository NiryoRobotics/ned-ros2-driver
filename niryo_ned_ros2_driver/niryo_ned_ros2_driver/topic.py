# /usr/bin/env python3

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
    ):
        self._node = node
        self._topic_name = topic_name
        self._topic_types = topic_types
        self._prefix = prefix
        self._qos = self._get_ros2_qos_for_topic(topic_name)
        self._rosbridge_client = rosbridge_client

        self._is_subscribed = False
        self._is_published = False

        self._node.get_logger().debug(
            f"Creating topic bridge for {topic_name} ({topic_types.ros1_type} → {topic_types.ros2_type})"
        )

        self._ros2_type_str = topic_types.ros2_type
        self._ros2_msg_class = get_message(self._ros2_type_str)

        self._loopback_filter = LoopbackFilter()

        self._ros1_publisher = self._create_ros1_publisher()
        self._ros2_publisher = self._create_ros2_publisher()

        self._ros2_subscriber = None
        self._ros1_subscriber = None

        self._node.create_timer(
            timer_period_sec=1.0, callback=self._subscription_manager_loop
        )

    def _subscription_manager_loop(self):
        """
        This function is called periodically to check the status of the
        subscribers and publishers for the ROS2 topic.
        It will create or destroy the ROS1 subscriber and the ROS2 subscriber
        depending on its status.
        """
        self._handle_subscriber()
        self._handle_publisher()

    def _handle_subscriber(self):
        """
        Check the subscriber count for the ROS2 topic.
        Once the topic is subscribed, instanciate the ROS1 subscriber.
        """
        num_subscribers = self._node.count_subscribers(
            f"{self._prefix}{self._topic_name}"
        )
        if num_subscribers >= 0 and not self._is_subscribed:
            self._ros1_subscriber = self._create_ros1_subscriber(self._ros1_publisher)
            self._is_subscribed = True
        elif num_subscribers == 0 and self._is_subscribed:
            self._ros1_publisher.unsubscribe()
            self._is_subscribed = False

    def _handle_publisher(self):
        """
        Check the publisher count for the ROS2 topic.
        Once the topic is published, instanciate the ROS2 subscriber.
        """
        num_publishers = self._node.count_publishers(
            f"{self._prefix}{self._topic_name}"
        )
        if num_publishers > 1 and not self._is_published:
            self._ros2_subscriber = self._create_ros2_subscriber()
            self._is_published = True
        elif num_publishers == 1 and self._is_published:
            self._node.destroy_subscription(self._ros2_subscriber)
            self._is_published = False

    def _create_ros2_publisher(self):
        """
        Create a ROS2 publisher for the topic.
        """
        return self._node.create_publisher(
            self._ros2_msg_class,
            f"{self._prefix}{self._topic_name}",
            self._qos,
        )

    def _create_ros2_subscriber(self):
        """
        Create a ROS2 subscriber for the topic.
        """
        return self._node.create_subscription(
            self._ros2_msg_class,
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
            normalize_ROS2_type_to_ROS1(msg_dict)
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
