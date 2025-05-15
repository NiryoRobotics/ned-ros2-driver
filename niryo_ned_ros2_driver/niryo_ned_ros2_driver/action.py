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
    def __init__(
        self,
        node: Node,
        action_name: str,
        action_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
        callback_group,
    ):
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
        Create a ROS1 action client.
        """
        return roslibpy.actionlib.ActionClient(
            self._rosbridge_client,
            f"{self._prefix}{self._action_name}",
            self._action_types.ros1_type,
        )

    def _create_ros2_action_server(self):
        """
        Create a ROS2 action server.
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
        Callback which executes the action.
        It sends the request to the ROS1 action server and wait for the result.
        """
        self._node.get_logger().debug(
            f"Executing action {self._action_name} of type {self._action_types.ros2_type} with command: {goal_handle.request}"
        )

        ros1_result = None
        result_received_event = threading.Event()

        request = ros2_message_to_dict(goal_handle.request)
        normalize_ROS2_type_to_ROS1(request, self._action_types.ros1_type)

        ros1_goal = roslibpy.actionlib.Goal(
            self._ros1_action_client,
            roslibpy.Message(request),
        )

        def feedback_callback(message):
            normalize_ROS1_type_to_ROS2(message, self._action_types.ros2_type)
            feedback_msg = self._ros2_action_class.Feedback()
            set_message_fields(feedback_msg, message)
            goal_handle.publish_feedback(feedback_msg)

        def result_callback(result):
            nonlocal ros1_result
            normalize_ROS1_type_to_ROS2(ros1_result, self._action_types.ros2_type)
            ros1_result = result
            result_received_event.set()

        ros1_goal.on("feedback", lambda message: feedback_callback(message))
        ros1_goal.send(result_callback=result_callback)

        while not result_received_event.is_set():
            if goal_handle.is_cancel_requested:
                ros1_goal.cancel()
            time.sleep(0.01)

        ros1_goal_status = ros1_goal.status["status"]
        ros2_result = self._ros2_action_class.Result()
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
        Callback which cancels the action.
        """
        self._node.get_logger().debug(
            f"Canceling action {self._action_name} of type {self._action_types.ros2_type}..."
        )

        return CancelResponse.ACCEPT
