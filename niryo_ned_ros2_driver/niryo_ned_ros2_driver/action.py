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

import threading
import time

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle

from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py.set_message import set_message_fields

import roslibpy
import roslibpy.actionlib

from .utils.models import ROSTypes
from .utils.conversion import (
    ros2_message_to_dict,
    normalize_ROS1_type_to_ROS2,
    normalize_ROS2_type_to_ROS1,
)


class Action:
    """
    Action class for bridging ROS1 and ROS2 action systems.

    This class creates a bidirectional bridge that allows ROS2 clients to call ROS1 action servers.
    It sets up a ROS2 action server that forwards requests to a ROS1 action client, handles the
    communication between the two systems, including goal sending, feedback publishing, and result handling.

    The class manages:
    - Creating the appropriate ROS1 action client
    - Setting up a ROS2 action server
    - Converting between ROS1 and ROS2 message formats
    - Translating action states between the two systems
    - Handling cancellation requests
    """

    def __init__(
        self,
        node: Node,
        action_name: str,
        action_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
        callback_group,
    ):
        """
        Initialize the Action class which serves as a bridge between ROS1 and ROS2 actions.

        This class creates a ROS2 action server that forwards action calls to a ROS1 action client,
        enabling communication between ROS1 and ROS2 systems.

        Args:
            node : Node
                The ROS2 node instance
            action_name : str
                Name of the action
            action_types : ROSTypes
                Container for ROS1 and ROS2 action type information
            prefix : str
                Prefix to be applied to the action name in ROS1
            rosbridge_client : roslibpy.Ros
                Client connection to the ROS1 bridge
            callback_group : rclpy.callback_groups.CallbackGroup
                Callback group for ROS2 action server
        """
        self._node = node
        self._action_name = action_name
        self._action_types = action_types
        self._prefix = prefix
        self._rosbridge_client = rosbridge_client
        self._callback_group = callback_group

        self._node.get_logger().debug(
            f"Creating action bridge for {action_name} ({action_types.ros1_type} → {action_types.ros2_type})"
        )

        self._ros2_action_class = get_action(action_types.ros2_type)

        self._ros1_action_client = self._create_ros1_action_client()
        self._ros2_action_server = self._create_ros2_action_server()

    def _create_ros1_action_client(self):
        """
        Creates and returns a ROS1 action client.

        This method instantiates a new roslibpy ActionClient that connects to
        the ROS1 action server through the rosbridge WebSocket connection.

        Returns:
            roslibpy.actionlib.ActionClient: A ROS1 action client configured with
                the appropriate action name and message types.
        """

        return roslibpy.actionlib.ActionClient(
            self._rosbridge_client,
            f"{self._action_name}",
            self._action_types.ros1_type,
        )

    def _create_ros2_action_server(self):
        """
        Create and return a ROS2 Action Server.

        This method initializes an ActionServer with the node, action class, action name,
        callback functions for execution and cancellation, and the callback group.

        Returns:
            ActionServer: A configured ROS2 ActionServer instance.
        """
        return ActionServer(
            self._node,
            self._ros2_action_class,
            f"{self._prefix}{self._action_name}",
            execute_callback=self._execute_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute the action when a goal is received.

        This method is called when a client sends a goal to the action server. It:
        1. Converts the ROS2 goal request to a ROS1 format
        2. Creates and sends a ROS1 action goal
        3. Sets up callbacks for feedback and result
        4. Handles goal cancellation requests
        5. Maps ROS1 terminal states to ROS2 terminal states
        6. Returns the final result to the client

        Args:
            goal_handle (ServerGoalHandle): The goal handle from the ROS2 action server,
                                          containing the client's request

        Returns:
            Result: The action result message of type defined by _ros2_action_class.Result()

        Note:
            This implementation bridges between ROS2 and ROS1 action protocols.
            The method blocks until the action result is received from the ROS1 side.
        """
        self._node.get_logger().debug(
            f"Executing action {self._action_name} of type {self._action_types.ros2_type} with command: {goal_handle.request}"
        )

        ros1_result = None
        ros2_result = self._ros2_action_class.Result()
        result_received_event = threading.Event()

        request = ros2_message_to_dict(goal_handle.request)
        normalize_ROS2_type_to_ROS1(request, self._action_types.ros1_type)

        ros1_goal = roslibpy.actionlib.Goal(
            self._ros1_action_client,
            roslibpy.Message(request),
        )

        def feedback_callback(message):
            feedback_msg = self._ros2_action_class.Feedback()
            normalize_ROS1_type_to_ROS2(
                message, feedback_msg.get_fields_and_field_types()
            )
            set_message_fields(feedback_msg, message)
            goal_handle.publish_feedback(feedback_msg)

        def result_callback(result):
            nonlocal ros1_result, ros2_result
            normalize_ROS1_type_to_ROS2(
                ros1_result, ros2_result.get_fields_and_field_types()
            )
            ros1_result = result
            result_received_event.set()

        ros1_goal.on("feedback", lambda message: feedback_callback(message))
        ros1_goal.send(result_callback=result_callback)

        while not result_received_event.is_set():
            if goal_handle.is_cancel_requested:
                ros1_goal.cancel()
            time.sleep(0.01)

        ros1_goal_status = ros1_goal.status["status"]
        set_message_fields(ros2_result, ros1_result)

        # Map ROS1 terminal states to ROS2 terminal states
        # https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html
        if ros1_goal_status == 3:
            goal_handle.succeed()
        elif ros1_goal_status in [4, 5]:
            goal_handle.abort()
        elif ros1_goal_status in [2, 8]:
            goal_handle.canceled()
        else:
            self._node.get_logger().warn(
                f"Unknown ROS1 goal status: {ros1_goal_status}"
            )
            goal_handle.abort()

        self._node.get_logger().debug(
            f"Action {self._action_name} of type {self._action_types.ros2_type} finished with status {ros1_goal_status}"
        )

        ros1_goal.remove_all_listeners()

        return ros2_result

    def _cancel_callback(self, goal_handle: ServerGoalHandle):
        """
        Cancel callback for the action server.
        This method is called when a client requests to cancel this action.
        It simply accepts all cancellation requests.
        Args:
            goal_handle (ServerGoalHandle): The goal handle associated with the cancellation request.
        Returns:
            CancelResponse.ACCEPT: Always accepts the cancellation request.
        """

        self._node.get_logger().debug(
            f"Canceling action {self._action_name} of type {self._action_types.ros2_type}..."
        )

        return CancelResponse.ACCEPT
