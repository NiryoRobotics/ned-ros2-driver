# /usr/bin/env python3

from rclpy.node import Node

import roslibpy
from concurrent.futures import ThreadPoolExecutor, as_completed
import time

from .topic import Topic
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
        whitelist_interfaces: list[str],
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

        if use_whitelist:
            self._node.get_logger().debug(
                f"Using whitelisted topics: {whitelist_interfaces}"
            )
            topic_names = whitelist_interfaces
        else:
            self._node.get_logger().debug("Retrieving topics...")
            topic_names, duration, label = execute_and_return_duration(
                "get_topics", self._rosbridge_client.get_topics
            )
            timings[label] = duration

        topic_type_map, duration, label = execute_and_return_duration(
            "get_topic_types_parallel", self._get_topic_types_parallel, topic_names
        )
        timings[label] = duration

        filtered_topics, duration, label = execute_and_return_duration(
            "filter_topics", filter_topics, topic_type_map
        )
        timings[label] = duration

        _, duration, label = execute_and_return_duration(
            "register_topics", self._register_topics, filtered_topics
        )
        timings[label] = duration

        self._node.get_logger().debug("=== Topic setup timings (in seconds) ===")
        for step, duration in timings.items():
            self._node.get_logger().debug(f"{step}: {duration:.3f}")

    def __del__(self):
        if self._rosbridge_client.is_connected:
            self._rosbridge_client.terminate()
            self._node.get_logger().info("ROSBridge connection closed.")

    def _get_topic_types_parallel(self, topic_names: list[str]) -> dict:
        """
        Get topic types in parallel using ThreadPoolExecutor.
        """
        topic_type_map = {}
        with ThreadPoolExecutor(max_workers=32) as executor:
            future_to_topic = {
                executor.submit(
                    self._safe_get_topic_type, self._rosbridge_client, topic
                ): topic
                for topic in topic_names
            }
            for future in as_completed(future_to_topic):
                topic = future_to_topic[future]
                try:
                    topic_type = future.result()
                    topic_type_map[topic] = topic_type
                except Exception as e:
                    self._node.get_logger().warn(f"Failed to get type for {topic}: {e}")
        return topic_type_map

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

            ros2_type = convert_ros1_to_ros2_type(ros1_type)

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

    def _safe_get_topic_type(self, client, topic, retries=3, delay=0.1):
        """
        Safely get the topic type with retries.
        This is useful for topics that may not be available immediately
        or if rosbridge gets too many requests at once.
        """
        for attempt in range(retries):
            try:
                topic_type = client.get_topic_type(topic)
                return topic_type
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(delay)
                else:
                    raise e
