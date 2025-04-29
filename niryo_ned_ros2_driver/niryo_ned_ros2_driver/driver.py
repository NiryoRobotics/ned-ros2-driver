# /usr/bin/env python3

from typing import Callable

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import roslibpy
from concurrent.futures import ThreadPoolExecutor, as_completed
import time

from .topic import Topic
from .service import Service
from .action import Action
from .tf_static_topic import StaticTFTopic
from .models import ROSTypes
from .utils.type_mapping import (
    convert_ros1_to_ros2_type,
    guess_action_type_from_goal_type,
)
from .utils.debug import execute_and_return_duration
from .utils.filtering import filter_topics, filter_services, filter_actions
from .utils.constants import ROS1_ACTIONS


class ROS2Driver:
    def __init__(
        self,
        node: Node,
        namespace: str,
        ip: str,
        port: int,
        topic_whitelist: list[str],
        service_whitelist: list[str],
        action_whitelist: list[str],
    ):
        self._node = node
        self._namespace = namespace
        self._rosbridge_client = roslibpy.Ros(host=ip, port=port)
        self._callback_group = ReentrantCallbackGroup()

        self._topics = []
        self._services = []
        self._actions = []

        timings = {}

        try:
            self._rosbridge_client.run()
        except roslibpy.core.RosTimeoutError:
            raise Exception(f"Timeout to connect to ROSBridge at ws://{ip}:{port}")

        # Get interfaces
        topics = self._get_topics(
            topic_whitelist=topic_whitelist,
            timings=timings,
        )

        services = self._get_services(
            service_whitelist=service_whitelist,
            timings=timings,
        )

        actions = self._get_actions(
            action_whitelist=action_whitelist,
            timings=timings,
        )

        # Register interfaces
        _, duration, label = execute_and_return_duration(
            "register_topics", self._register_topics, topics
        )
        timings[label] = duration

        _, duration, label = execute_and_return_duration(
            "register_services", self._register_services, services
        )
        timings[label] = duration

        _, duration, label = execute_and_return_duration(
            "register_actions", self._register_actions, actions
        )
        timings[label] = duration

        self._node.get_logger().debug("=== Topic setup timings (in seconds) ===")
        for step, duration in timings.items():
            self._node.get_logger().debug(f"{step}: {duration:.3f}")

        self._topic_management_timer = self._node.create_timer(
            1.0,
            self._manage_topics,
            callback_group=self._callback_group,
        )

    def __del__(self):
        if self._rosbridge_client.is_connected:
            self._rosbridge_client.terminate()
            self._node.get_logger().info("ROSBridge connection closed.")

    def _manage_topics(self):
        """
        This function is called periodically to manage topics subscriptions.
        """
        for topic in self._topics:
            topic.update()

    def _get_action_type(self, action_name: str) -> str:
        """
        Given an action name, return the corresponding action type.
        """
        goal_topic = action_name + "/goal"
        goal_type = self._rosbridge_client.get_topic_type(goal_topic)
        return guess_action_type_from_goal_type(goal_type)

    def _get_actions(self, action_whitelist: list[str] = [], timings: dict = {}):
        """
        Get all the actions available on the ROS1 side and filter them.

        Args:
            action_whitelist (list[str]): List of regex for whitelisted actions.
            timings (dict): Dictionary to store timings for each step. Only for debug purpose.
        Returns:
            Actions (dict[str, str]): Dictionnary with keys as action names and values as their types.
        """
        self._node.get_logger().debug("Retrieving actions...")

        # Get the types of the actions
        action_type_map, duration, label = execute_and_return_duration(
            "get_action_types_parallel",
            self._get_interface_types_parallel,
            ROS1_ACTIONS,
            self._get_action_type,
        )
        timings[label] = duration

        # Filter the actions to remove excluded ones
        filtered_actions, duration, label = execute_and_return_duration(
            "filter_actions", filter_actions, action_type_map, action_whitelist
        )
        timings[label] = duration

        return filtered_actions

    def _register_actions(self, actions: dict):
        """
        Get all the actions available on the ROS1 side and create the corresponding
        actions on the ROS2 side.
        """
        self._node.get_logger().debug("Registering actions...")
        for action_name, ros1_type in actions.items():
            self._node.get_logger().debug(
                f"Registering action {action_name} of type {ros1_type}"
            )

            ros2_type = convert_ros1_to_ros2_type(ros1_type, "action")

            action_type = ROSTypes(ros1_type=ros1_type, ros2_type=ros2_type)

            self._actions.append(
                Action(
                    self._node,
                    action_name,
                    action_type,
                    self._namespace,
                    self._rosbridge_client,
                    self._callback_group,
                )
            )

    def _get_services(
        self,
        service_whitelist: list[str] = [],
        timings: dict = {},
    ):
        """
        Get all the services available on the ROS1 side and filter them.

        Args:
            service_whitelist (list[str]): List of regex for whitelisted services.
            timings (dict): Dictionary to store timings for each step. Only for debug purpose.
        Returns:
            Services (dict[str, str]): Dictionnary with keys as service names and values as their types.
        """
        self._node.get_logger().debug("Retrieving services...")
        service_names, duration, label = execute_and_return_duration(
            "get_services", self._rosbridge_client.get_services
        )
        timings[label] = duration

        # Get the types of the services
        service_type_map, duration, label = execute_and_return_duration(
            "get_service_types_parallel",
            self._get_interface_types_parallel,
            service_names,
            self._rosbridge_client.get_service_type,
        )
        timings[label] = duration

        # Filter the topics to remove excluded ones
        filtered_services, duration, label = execute_and_return_duration(
            "filter_services", filter_services, service_type_map, service_whitelist
        )
        timings[label] = duration

        return filtered_services

    def _register_services(self, services: dict):
        """
        Get all the services available on the ROS1 side and create the corresponding
        services on the ROS2 side.
        """
        self._node.get_logger().debug("Registering services...")
        for service_name, ros1_type in services.items():
            self._node.get_logger().debug(
                f"Registering service {service_name} of type {ros1_type}"
            )

            ros2_type = convert_ros1_to_ros2_type(ros1_type, "srv")

            service_type = ROSTypes(ros1_type=ros1_type, ros2_type=ros2_type)

            self._services.append(
                Service(
                    self._node,
                    service_name,
                    service_type,
                    self._namespace,
                    self._rosbridge_client,
                    self._callback_group,
                )
            )

    def _get_topics(
        self,
        topic_whitelist: list[str] = [],
        timings: dict = {},
    ):
        """
        Get all the topics available on the ROS1 side and filter them.

        Args:
            topic_whitelist (list[str]): List of regex for whitelisted topics.
            timings (dict): Dictionary to store timings for each step. Only for debug purpose.
        Returns:
            Topics (dict[str, str]): Dictionary with keys as topic names and values as their types.
        """
        self._node.get_logger().debug("Retrieving topics...")
        topic_names, duration, label = execute_and_return_duration(
            "get_topics", self._rosbridge_client.get_topics
        )
        timings[label] = duration

        # Get the types of the topics
        topic_type_map, duration, label = execute_and_return_duration(
            "get_topic_types_parallel",
            self._get_interface_types_parallel,
            topic_names,
            self._rosbridge_client.get_topic_type,
        )
        timings[label] = duration

        # Filter the topics to remove excluded ones
        filtered_topics, duration, label = execute_and_return_duration(
            "filter_topics", filter_topics, topic_type_map, topic_whitelist
        )
        timings[label] = duration

        return filtered_topics

    def _get_interface_types_parallel(
        self,
        interface_names: list[str],
        interface_type_getter: Callable,
    ):
        """
        Get interface types in parallel using ThreadPoolExecutor.
        """
        interface_type_map = {}
        with ThreadPoolExecutor() as executor:
            future_to_interface = {
                executor.submit(
                    self._safe_get_type, interface_type_getter, interface
                ): interface
                for interface in interface_names
            }
            for future in as_completed(future_to_interface):
                interface = future_to_interface[future]
                try:
                    interface_type = future.result()
                    interface_type_map[interface] = interface_type
                except Exception as e:
                    self._node.get_logger().warn(
                        f"Failed to get type for {interface}: {e}"
                    )
        return interface_type_map

    def _register_topics(self, topics: dict):
        """
        Get all the topics available on the ROS1 side and create the corresponding
        topics on the ROS2 side.
        """
        self._node.get_logger().debug("Registering topics...")
        for topic_name, ros1_type in topics.items():
            self._node.get_logger().debug(
                f"Registering topic {topic_name} of type {ros1_type}"
            )

            ros2_type = convert_ros1_to_ros2_type(ros1_type, "msg")

            topic_type = ROSTypes(ros1_type=ros1_type, ros2_type=ros2_type)

            if topic_name == "/tf_static":
                # Special case for static TFs
                self._topics.append(
                    StaticTFTopic(
                        self._node,
                        topic_name,
                        topic_type,
                        self._namespace,
                        self._rosbridge_client,
                        self._callback_group,
                    )
                )
            else:
                self._topics.append(
                    Topic(
                        self._node,
                        topic_name,
                        topic_type,
                        self._namespace,
                        self._rosbridge_client,
                        self._callback_group,
                    )
                )

    def _safe_get_type(
        self, type_getter: Callable, interface_name: str, retries=3, delay=0.1
    ):
        """
        Safely get the type with retries.
        This is useful for interfaces (topics, services) that may not be available immediately
        or if rosbridge gets too many requests at once.

        Args:
            type_getter (Callable): Function to get the type.
            interface_name (str): Name of the interface (topic/service).
            retries (int): Number of retries.
            delay (float): Delay between retries.
        Returns:
            type: The type of the interface.
        """
        for attempt in range(retries):
            try:
                return type_getter(interface_name)
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(delay)
                else:
                    raise e
