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

from .utils.models import ROSTypes
from .utils.conversion import (
    ros2_message_to_dict,
    normalize_ROS1_type_to_ROS2,
    normalize_ROS2_type_to_ROS1,
)
from .utils.constants import LATCHED_ROS1_TOPICS
from .utils.loopback_filter import LoopbackFilter


class Topic:
    """
    A bidirectional bridge for topics between ROS1 and ROS2.

    This class implements a bridge that enables communication between ROS1 and ROS2 for a specific topic.
    It handles message conversion, topic subscription and publication management,
    and ensures proper message routing between the two ROS ecosystems.

        The class manages:
        - Bidirectional communication between ROS1 and ROS2 topics
        - Automatic handling of topic subscriptions based on publisher/subscriber presence
        - Message format conversion between ROS1 and ROS2
        - Loopback filtering to prevent message cycles
        - Configurable QoS profiles based on topic characteristics
    """

    def __init__(
        self,
        node: Node,
        topic_name: str,
        topic_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
        callback_group,
    ):
        """
        Initialize a Topic bridge between ROS1 and ROS2.

        This class creates a bidirectional bridge for a specific topic between ROS1 (through rosbridge)
        and ROS2, handling the message conversions and communication.

        Args:
            node (Node): The ROS2 node instance for creating publishers/subscribers.
            topic_name (str): Name of the topic to bridge.
            topic_types (ROSTypes): Container for ROS1 and ROS2 message type information.
            prefix (str): Prefix to use for the bridged topic names.
            rosbridge_client (roslibpy.Ros): Connection to the ROS1 system via rosbridge.
            callback_group: The callback group for ROS2 publishers/subscribers.
        """
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

        # Get the field types for the response
        self._message_field_types = self._ros2_msg_class.get_fields_and_field_types()

        self._loopback_filter = LoopbackFilter()

        self._ros1_publisher = self._create_ros1_publisher()
        self._ros2_publisher = self._create_ros2_publisher()

        self._ros2_subscriber = None
        self._ros1_subscriber = None

    @property
    def full_topic_name(self):
        """
        Get the full topic name including prefix.

        Returns:
            str: The full topic name as a concatenation of the prefix and the topic name.
        """
        return f"{self._prefix}{self._topic_name}"

    def update(self):
        """
        Update the topic handlers based on the current state of the ROS graph.

        Checks for changes in the publishers and subscribers to the topic.
        If there are changes compared to the previous state, it updates the
        subscription and publication handlers accordingly.
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
        Update the subscription state based on the number of ROS2 subscribers.

        This method checks the current number of ROS2 subscribers and manages the ROS1
        subscription accordingly:
        - If there's at least one ROS2 subscriber and we're not currently subscribed to ROS1,
          create a ROS1 subscriber and set the subscribed flag to True.
        - If there are no ROS2 subscribers and we're currently subscribed to ROS1,
          unsubscribe from ROS1 and set the subscribed flag to False

        This approach ensures that ROS1 messages are only processed when there are
        active ROS2 subscribers, optimizing resource usage.
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
        Update the publication status of the topic based on the number of publishers in the ROS graph.

        This method checks if there are multiple publishers for this topic in the ROS graph.
        - If there are more than one publisher and the topic is not currently subscribed,
            it creates a ROS2 subscriber and sets the publication flag to True.
        - If there are one or zero publishers and the topic is currently subscribed,
            it destroys the existing publication and sets the publication flag to False.

        The method helps optimize resource usage by only subscribing to topics when there
        are actual publishers available.
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
        Create a ROS2 publisher for this topic.

        Returns:
            rclpy.publisher.Publisher: A ROS2 publisher instance configured with the specified parameters.
        """

        return self._node.create_publisher(
            self._ros2_msg_class,
            self.full_topic_name,
            self._qos,
        )

    def _create_ros2_subscriber(self):
        """
        Creates and returns a ROS2 subscription for the topic.ce.

        Returns:
            rclpy.subscription.Subscription: The created ROS2 subscription object.
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
        Create a ROS1 Publisher using roslibpy.

        This method creates a ROS1 topic using the roslibpy library that
        connects to the ROS1 topic through the rosbridge WebSocket connection.

        Returns:
            roslibpy.Topic: A topic instance which acts as a publisher in ROS1.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        )

    def _create_ros1_subscriber(self, topic: roslibpy.Topic):
        """
        Create a ROS1 topic.

        This method creates a ROS1 topic using the roslibpy library that
        connects to the ROS1 topic through the rosbridge WebSocket connection.

        Args:
            topic : roslibpy.Topic
                The ROS1 topic to subscribe to

        Returns:
            roslibpy.Subscription
                A subscription object which can be used to unsubscribe later
        """
        return topic.subscribe(self._ros1_callback)

    def _ros1_callback(self, ros1_msg_dict):
        """
        Callback function for processing ROS1 messages received through the bridge.

        This method handles ROS1 messages by:
        1. Normalizing them to match ROS2 format using the expected ROS2 message type
        2. Filtering out loopback messages to prevent message cycling between ROS1 and ROS2
        3. Converting and publishing the message to the ROS2 network

        Args:
            ros1_msg_dict : dict
                Dictionary representation of the ROS1 message received from the bridge
        """

        # Normalize the ROS1 message to match the expected ROS2 format
        normalize_ROS1_type_to_ROS2(ros1_msg_dict, self._message_field_types)

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
        Callback function for ROS2 subscriber.

        This method handles ROS2 messages by:
        1. Converting the ROS2 message to a dictionary format
        2. Normalizing the message to match the expected ROS1 format
        3. Filtering out loopback messages to prevent message cycling between ROS2 and ROS1
        4. Publishing the message to the ROS1 network

        Args:
            ros2_msg: The ROS2 message received from a subscription.
        """
        try:
            msg_dict = ros2_message_to_dict(ros2_msg)

            # Check if the message hash is cached, cache it and forward it if not
            if not self._loopback_filter.should_forward(msg_dict):
                return

            # Normalize the ROS2 message to match the expected ROS1 format after the loopback check
            # This is important to ensure that the message format are similar for the hash comparison
            # in the loopback filter
            normalize_ROS2_type_to_ROS1(
                msg_dict,
            )
            self._ros1_publisher.publish(msg_dict)
        except Exception as e:
            self._node.get_logger().error(
                f"Failed to convert ROS2 → ROS1 message for topic '{self._topic_name}': {e}"
            )

    def _get_ros2_qos_for_topic(self, topic_name: str) -> QoSProfile:
        """
        Determines the appropriate QoS (Quality of Service) profile for a ROS2 topic based on its name.

        This method checks if the given topic name is in the list of latched ROS1 topics and returns
        a QoS profile that mimics ROS1 latched behavior (using RELIABLE reliability and TRANSIENT_LOCAL
        durability) if it is. Otherwise, it returns a default QoS profile with RELIABLE reliability
        and VOLATILE durability.

        Args:
            topic_name : str
                The name of the ROS2 topic to get a QoS profile for

        Returns:
            rclpy.qos.QoSProfile
                A QoS profile configured appropriately for the given topic name
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
