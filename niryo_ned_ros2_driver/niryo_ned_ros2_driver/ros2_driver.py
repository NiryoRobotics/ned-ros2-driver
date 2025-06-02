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
from .utils.models import ROSTypes
from .utils.type_mapping import (
    convert_ros1_to_ros2_type,
    guess_action_type_from_goal_type,
)
from .utils.debug import execute_and_return_duration
from .utils.filtering import filter_topics, filter_services, filter_actions
from .utils.constants import ROS1_ACTIONS


class ROS2Driver:
    """
    A ROS2 driver that bridges communication with a Niryo Ned ROS1 system via ROSBridge.
    This class manages the connection between a ROS2 node and a ROS1 system by:
    1. Connecting to a ROSBridge server
    2. Discovering available interfaces (topics, services, actions)
    3. Filtering interfaces based on provided whitelists
    4. Creating and managing appropriate bridging objects

    The driver handles topics (including special cases like TF), services, and actions,
    automatically converting between ROS1 and ROS2 message/service/action types.
    """

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
        """
        Initialize a ROS 2 driver that connects to a ROSBridge server and sets up topics, services, and actions.

        This constructor establishes a connection to a ROSBridge websocket server, fetches the available
        interfaces based on provided whitelists, and registers them to enable communication between
        ROS 1 and ROS 2 systems.

        Args:
            node : Node
                The ROS 2 node that will own the driver
            namespace : str
                Namespace prefix for all registered interfaces
            ip : str
                IP address of the ROSBridge server
            port : int
                Port number of the ROSBridge server
            topic_whitelist : list[str]
                List of topic patterns to include for bridging
            service_whitelist : list[str]
                List of service patterns to include for bridging
            action_whitelist : list[str]
                List of action patterns to include for bridging

        Raises:
            Exception
                If connection to ROSBridge server times out
        """
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

    def disconnect(self):
        """
        Disconnect from the robot by terminating the rosbridge client connection and destroying the topic management timer.

        This method ensures that all resources are properly released when the connection to the robot is no longer needed.
        It first checks if the rosbridge client is connected and terminates it if so, then destroys the topic management
        timer if it exists.
        """
        if self._rosbridge_client.is_connected:
            self._rosbridge_client.terminate()
        if self._topic_management_timer:
            self._node.destroy_timer(self._topic_management_timer)
            self._topic_management_timer = None

    def _manage_topics(self):
        """
        Updates all topics in the ROS2 driver.

        This method iterates through the list of topics
        and calls the update method on each topic to refresh its state.
        """
        for topic in self._topics:
            topic.update()

    def _get_action_type(self, action_name: str) -> str:
        """
        Determines the action type based on the action name.

        This method retrieves the goal topic type for a given action name from the
        ROS bridge client and uses it to infer the action type.

        Args:
            action_name (str): The name of the action to get the type for.

        Returns:
            str: The inferred action type.
        """
        goal_topic = action_name + "/goal"
        goal_type = self._rosbridge_client.get_topic_type(goal_topic)
        return guess_action_type_from_goal_type(goal_type)

    def _get_actions(self, action_whitelist: list[str] = [], timings: dict = {}):
        """
        Retrieve and filter the available ROS1 actions.

        This method gets the types of available ROS1 actions and filters them according to a whitelist.

        Args:
            action_whitelist (list[str], optional): List of actions to include. If empty, all actions are included.
                Defaults to [].
            timings (dict, optional): Dictionary to store timing information about the execution.
                Timing data will be added to this dictionary. Defaults to {}.

        Returns:
            dict: A dictionary mapping action names to their types, filtered according to the whitelist.

        Note:
            This method logs at the debug level when it starts retrieving actions.
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
        Register ROS1 actions and convert them to ROS2 actions.

        This method iterates through the provided dictionary of actions, converting each ROS1 action type
        to its corresponding ROS2 type. For each action, it creates an Action object that handles the
        communication between ROS1 and ROS2.

        Args:
            actions (dict): A dictionary mapping action names (str) to their ROS1 type names (str)
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
        Retrieves services from ROS and filters them based on a whitelist.

        This method fetches all available ROS services, their types, and filters them
        according to the provided whitelist.

        Args:
            service_whitelist (list[str], optional): List of service name patterns to include.
                Services matching any pattern in this list will be included.
                If empty, no filtering is applied. Defaults to [].
            timings (dict, optional): Dictionary to store execution duration of different
                stages of the method. Keys will be the stage labels, values will be the
                durations. Defaults to {}.

        Returns:
            dict: A dictionary mapping service names to their types, filtered according
                to the whitelist.

        Note:
            The method measures and records the execution time of each stage if a timing
            dictionary is provided.
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
        Register ROS 2 services that correspond to ROS 1 services.

        This method creates ROS 2 services that will communicate with the ROS 1 bridge.
        For each service in the provided dictionary, it converts the ROS 1 service type
        to its ROS 2 equivalent and creates a Service instance that handles the communication.

        Args:
            services (dict): A dictionary mapping service names (str) to their ROS 1 service types (str).
                             For example: {'calibrate_motors': 'niryo_robot_msgs/Trigger'}
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
        Retrieve topics from the ROS network.

        This method retrieves all available topics, gets their types, and filters them
        based on an optional whitelist.

        Args:
            topic_whitelist (list[str], optional): A list of topic names to include.
                If empty, all topics will be considered. Defaults to [].
            timings (dict, optional): A dictionary to store execution durations of
                different steps in the process. Defaults to {}.

        Returns:
            dict: A filtered dictionary mapping topic names to their types.

        Note:
            The method logs debug information and tracks the execution time of each step.
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
        Get types of multiple interfaces in parallel.

        This method retrieves interface types (for topics, services, or actions) in parallel
        using multiple threads to improve performance when fetching information for multiple
        interfaces.

        Args:
            interface_names : list[str]
                List of interface names (topics, services, or actions) to get types for.
            interface_type_getter : Callable
                Function that retrieves the type for a single interface name.
                This should be a method like `_node.get_topic_type`.

        Returns:
            dict[str, str]
                A mapping from interface names to their corresponding types.
                If an interface's type couldn't be retrieved, it will be excluded from the result.
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
        Register ROS topics for communication between ROS1 and ROS2.
        This method creates appropriate Topic objects based on the given topics dictionary.
        It handles special cases like static transforms (/tf_static) differently.

        Args:
            topics : dict
                Dictionary mapping topic names (str) to ROS1 message types (str).
                Example: {"/joint_states": "sensor_msgs/JointState"}
        Note:
            - The method converts ROS1 message types to equivalent ROS2 types
            - For most topics, it creates standard Topic objects
            - For "/tf_static", it creates a specialized StaticTFTopic object
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
        Safely attempts to get a type from a given interface.

        This method attempts to get a type using the provided function, and if it fails,
        it will retry up to a specified number of times with a delay between attempts.

        Parameters
        ----------
        type_getter : Callable
            The function to call to get the type.
        interface_name : str
            The name of the interface to get the type from.
        retries : int, optional
            The number of attempts to make before giving up, by default 3.
        delay : float, optional
            The time in seconds to wait between attempts, by default 0.1.

        Returns
        -------
        Any
            The type retrieved from the interface.

        Raises
        ------
        Exception
            The exception from the last failed attempt if all attempts fail.
        """
        for attempt in range(retries):
            try:
                return type_getter(interface_name)
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(delay)
                else:
                    raise e
