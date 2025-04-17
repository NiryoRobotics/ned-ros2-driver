# /usr/bin/env python3

from typing import Callable

from rclpy.node import Node

import roslibpy
from concurrent.futures import ThreadPoolExecutor, as_completed
import time

from .topic import Topic
from .service import Service
from .tf_static_topic import StaticTFTopic
from .models import ROSTypes
from .utils.type_mapping import convert_ros1_to_ros2_type
from .utils.debug import execute_and_return_duration
from .utils.filtering import filter_topics


class ROS2Driver:
    def __init__(
        self,
        node: Node,
        namespace: str,
        ip: str,
        port: int,
        use_whitelist: bool,
        whitelist_topics: list[str],
        whitelist_services: list[str],
    ):
        self._node = node
        self._namespace = namespace
        self._rosbridge_client = roslibpy.Ros(host=ip, port=port)

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
            use_whitelist=use_whitelist,
            whitelist_topics=whitelist_topics,
            timings=timings,
        )

        services = self._get_services(
            use_whitelist=use_whitelist,
            whitelist_services=whitelist_services,
            timings=timings,
        )

        # Register interfaces
        # _, duration, label = execute_and_return_duration(
        #     "register_topics", self._register_topics, topics
        # )
        # timings[label] = duration

        # _, duration, label = execute_and_return_duration(
        #     "register_services", self._register_services, services
        # )

        self._node.get_logger().debug("=== Topic setup timings (in seconds) ===")
        for step, duration in timings.items():
            self._node.get_logger().debug(f"{step}: {duration:.3f}")

    def __del__(self):
        if self._rosbridge_client.is_connected:
            self._rosbridge_client.terminate()
            self._node.get_logger().info("ROSBridge connection closed.")

    def _get_services(
        self,
        use_whitelist: bool = False,
        whitelist_services: list[str] = [],
        timings: dict = {},
    ):
        """
        Get all the services available on the ROS1 side or use the whitelist.

        Args:
            use_whitelist (bool): Whether to use the whitelist.
            whitelist_services (list[str]): List of whitelisted services.
            timings (dict): Dictionary to store timings for each step. Only for debug purpose.
        Returns:
            Services (dict[str, str]): Dictionnary with keys as service names and values as their types.
        """
        # Get all the services available on the ROS1 side or use the whitelist
        if use_whitelist:
            self._node.get_logger().debug(
                f"Using whitelisted services: {whitelist_services}"
            )
            service_names = whitelist_services
        else:
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

        return service_type_map

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
                )
            )

    def _get_topics(
        self,
        use_whitelist: bool = False,
        whitelist_topics: list[str] = [],
        timings: dict = {},
    ):
        """
        Get all the topics available on the ROS1 side or use the whitelist
        and filter them to remove excluded ones.

        Args:
            use_whitelist (bool): Whether to use the whitelist.
            whitelist_topics (list[str]): List of whitelisted topics.
            timings (dict): Dictionary to store timings for each step. Only for debug purpose.
        Returns:
            Topics (dict[str, str]): Dictionary with keys as topic names and values as their types.
        """
        # Get all the topics available on the ROS1 side or use the whitelist
        if use_whitelist:
            self._node.get_logger().debug(
                f"Using whitelisted topics: {whitelist_topics}"
            )
            topic_names = whitelist_topics
        else:
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
            "filter_topics", filter_topics, topic_type_map
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
        with ThreadPoolExecutor(max_workers=32) as executor:
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
