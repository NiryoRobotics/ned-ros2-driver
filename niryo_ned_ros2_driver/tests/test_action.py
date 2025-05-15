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

import pytest
import roslibpy
from unittest.mock import MagicMock, patch
import threading
import time

from rclpy.action import CancelResponse

from niryo_ned_ros2_driver.action import Action
from niryo_ned_ros2_driver.utils.models import ROSTypes


class TestAction:
    """Test suite for the Action class that bridges ROS1 and ROS2 actions."""

    @pytest.fixture
    def mock_roslibpy_action_class(self):
        """Mock the roslibpy.actionlib.ActionClient class."""
        with patch(
            "niryo_ned_ros2_driver.action.roslibpy.actionlib.ActionClient"
        ) as mock:
            yield mock

    @pytest.fixture
    def mock_action_server(self):
        """Mock the rclpy.action.ActionServer class."""
        with patch("niryo_ned_ros2_driver.action.ActionServer") as mock:
            yield mock

    @pytest.fixture
    def mock_get_action(self):
        """Create a mock for get_action function."""
        with patch("niryo_ned_ros2_driver.action.get_action") as mock:
            yield mock

    @pytest.fixture
    def mock_ros1_action_client(self):
        """Create a mock ROS1 action client."""
        mock = MagicMock(spec=roslibpy.actionlib.ActionClient)
        # Mock the Goal class that will be instantiated within the Action class
        mock_goal = MagicMock()
        mock_goal.send.side_effect = lambda result_callback: result_callback(
            {"result": "success"}
        )
        mock_goal.status = {"status": 3}  # 3 is SUCCEEDED in ROS1

        # Make the ActionClient return our mock goal when Goal is created
        with patch("roslibpy.actionlib.Goal", return_value=mock_goal):
            yield mock

    @pytest.fixture
    def action_ros_types(self):
        """Create a ROSTypes fixture for action types."""
        return ROSTypes(
            ros1_type="control_msgs/FollowJointTrajectoryAction",
            ros2_type="control_msgs/action/FollowJointTrajectory",
        )

    @pytest.fixture
    def action_instance(
        self,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        action_ros_types,
        mock_get_action,
        mock_roslibpy_action_class,
        mock_action_server,
    ):
        """Create an Action instance for testing."""
        # Configure mocks
        mock_action_class = MagicMock()
        mock_get_action.return_value = mock_action_class

        mock_ros1_action_client = MagicMock()
        mock_roslibpy_action_class.return_value = mock_ros1_action_client

        mock_ros2_acion_server = MagicMock()
        mock_action_server.return_value = mock_ros2_acion_server

        # Create action instance
        action = Action(
            node=mock_node,
            action_name="/test_action",
            action_types=action_ros_types,
            prefix="",
            rosbridge_client=mock_rosbridge,
            callback_group=mock_callback_group,
        )

        return action

    def test_initialization(
        self,
        action_instance,
        mock_node,
        mock_rosbridge,
        mock_callback_group,
        action_ros_types,
        mock_get_action,
        mock_roslibpy_action_class,
        mock_action_server,
    ):
        """Test correct initialization of the Action."""
        # Verify initialization
        assert action_instance._node == mock_node
        assert action_instance._action_name == "/test_action"
        assert action_instance._action_types == action_ros_types
        assert action_instance._prefix == ""
        assert action_instance._rosbridge_client == mock_rosbridge
        assert action_instance._callback_group == mock_callback_group

        # Verify action class was fetched
        mock_get_action.assert_called_once_with(action_ros_types.ros2_type)

        # Verify ROS1 action client was created
        mock_roslibpy_action_class.assert_called_once_with(
            mock_rosbridge, "/test_action", action_ros_types.ros1_type
        )

        mock_action_server.assert_called_once_with(
            mock_node,
            mock_get_action.return_value,
            "/test_action",
            execute_callback=action_instance._execute_callback,
            cancel_callback=action_instance._cancel_callback,
            callback_group=mock_callback_group,
        )

    @patch("roslibpy.actionlib.Goal")
    def test_execute_callback_success(self, mock_goal_class, action_instance):
        """Test successful execution of an action."""
        # Create a mock goal handle
        goal_handle = MagicMock()
        goal_handle.request = MagicMock()
        goal_handle.is_cancel_requested = False

        # Mock the ros2_message_to_dict function
        with patch("niryo_ned_ros2_driver.action.ros2_message_to_dict") as mock_to_dict:
            mock_to_dict.return_value = {"goal": "test goal"}

            # Mock the Goal instantiation and behavior
            mock_goal = MagicMock()
            mock_goal_class.return_value = mock_goal

            # Mock the goal's send method to call the result_callback
            def mock_send_with_callback(result_callback):
                # Wait a short time then deliver the result
                threading.Timer(
                    0.05, lambda: result_callback({"result": "success"})
                ).start()

            mock_goal.send.side_effect = mock_send_with_callback
            mock_goal.status = {"status": 3}  # 3 = SUCCEEDED in ROS1

            # Mock normalization
            with patch("niryo_ned_ros2_driver.action.normalize_ROS2_type_to_ROS1"):
                with patch("niryo_ned_ros2_driver.action.normalize_ROS1_type_to_ROS2"):
                    with patch(
                        "niryo_ned_ros2_driver.action.set_message_fields"
                    ) as mock_set_fields:
                        # Execute the callback
                        action_instance._execute_callback(goal_handle)

                        # Verify the ROS1 goal was created and sent
                        mock_goal_class.assert_called_once()
                        mock_goal.send.assert_called_once()

                        # Verify the result was processed
                        goal_handle.succeed.assert_called_once()
                        mock_set_fields.assert_called_once()

                        # Verify event listeners were cleaned up
                        mock_goal.remove_all_listeners.assert_called_once()

    @patch("roslibpy.actionlib.Goal")
    def test_execute_callback_aborted(self, mock_goal_class, action_instance):
        """Test an action that is aborted by the ROS1 side."""
        # Create a mock goal handle
        goal_handle = MagicMock()
        goal_handle.request = MagicMock()
        goal_handle.is_cancel_requested = False

        # Set up the mock Goal
        mock_goal = MagicMock()
        mock_goal_class.return_value = mock_goal

        # Mock the goal's send method to call the result_callback
        def mock_send_with_callback(result_callback):
            threading.Timer(0.05, lambda: result_callback({"error": "failed"})).start()

        mock_goal.send.side_effect = mock_send_with_callback
        mock_goal.status = {"status": 4}  # 4 = ABORTED in ROS1

        # Mock conversions
        with patch("niryo_ned_ros2_driver.action.ros2_message_to_dict"):
            with patch("niryo_ned_ros2_driver.action.normalize_ROS2_type_to_ROS1"):
                with patch("niryo_ned_ros2_driver.action.normalize_ROS1_type_to_ROS2"):
                    with patch("niryo_ned_ros2_driver.action.set_message_fields"):
                        # Execute the callback
                        action_instance._execute_callback(goal_handle)

                        # Verify the goal was aborted
                        goal_handle.abort.assert_called_once()
                        goal_handle.succeed.assert_not_called()
                        goal_handle.canceled.assert_not_called()

    @patch("roslibpy.actionlib.Goal")
    def test_execute_callback_feedback(self, mock_goal_class, action_instance):
        """Test an action that provides feedback during execution."""
        # Create a mock goal handle
        goal_handle = MagicMock()
        goal_handle.request = MagicMock()
        goal_handle.is_cancel_requested = False

        # Set up the mock Goal
        mock_goal = MagicMock()
        mock_goal_class.return_value = mock_goal

        # Store the feedback callback for later use
        feedback_callback = None

        def store_callback(callback_name, callback_func):
            nonlocal feedback_callback
            if callback_name == "feedback":
                feedback_callback = callback_func

        mock_goal.on.side_effect = store_callback

        # Mock the goal's send method to call the result_callback after delay
        def mock_send_with_callback(result_callback):
            threading.Timer(0.2, lambda: result_callback({"result": "success"})).start()

        mock_goal.send.side_effect = mock_send_with_callback
        mock_goal.status = {"status": 3}  # 3 = SUCCEEDED in ROS1

        # Mock conversions
        with patch("niryo_ned_ros2_driver.action.ros2_message_to_dict"):
            with patch("niryo_ned_ros2_driver.action.normalize_ROS2_type_to_ROS1"):
                with patch("niryo_ned_ros2_driver.action.normalize_ROS1_type_to_ROS2"):
                    with patch("niryo_ned_ros2_driver.action.set_message_fields"):
                        # Start executing the callback in a thread
                        thread = threading.Thread(
                            target=action_instance._execute_callback,
                            args=(goal_handle,),
                        )
                        thread.daemon = True
                        thread.start()

                        # Let the execution start
                        time.sleep(0.05)

                        # Send feedback if we captured the callback
                        if feedback_callback:
                            feedback_callback({"progress": 50})

                            # Verify feedback was published
                            goal_handle.publish_feedback.assert_called_once()

                        # Wait for thread to complete
                        thread.join(timeout=0.5)

    @patch("roslibpy.actionlib.Goal")
    def test_execute_callback_cancel(self, mock_goal_class, action_instance):
        """Test cancellation of an action."""
        # Create a mock goal handle that requests cancellation
        goal_handle = MagicMock()
        goal_handle.request = MagicMock()

        # Set cancel_requested to True after a short delay
        goal_handle.is_cancel_requested = False

        def set_cancel_requested():
            goal_handle.is_cancel_requested = True

        threading.Timer(0.1, set_cancel_requested).start()

        # Set up the mock Goal
        mock_goal = MagicMock()
        mock_goal_class.return_value = mock_goal

        # Mock the goal's send method to call the result_callback after cancel
        def mock_send_with_delayed_result(result_callback):
            def send_result():
                mock_goal.status = {"status": 2}  # 2 = CANCELED in ROS1
                result_callback({"result": "canceled"})

            threading.Timer(0.2, send_result).start()

        mock_goal.send.side_effect = mock_send_with_delayed_result

        # Mock conversions
        with patch("niryo_ned_ros2_driver.action.ros2_message_to_dict"):
            with patch("niryo_ned_ros2_driver.action.normalize_ROS2_type_to_ROS1"):
                with patch("niryo_ned_ros2_driver.action.normalize_ROS1_type_to_ROS2"):
                    with patch("niryo_ned_ros2_driver.action.set_message_fields"):
                        # Execute the callback
                        action_instance._execute_callback(goal_handle)

                        # Verify the goal was canceled
                        mock_goal.cancel.assert_called()
                        goal_handle.canceled.assert_called_once()

    def test_cancel_callback(self, action_instance):
        """Test the cancel callback."""
        # Create a mock goal handle
        goal_handle = MagicMock()

        # Call the cancel callback
        result = action_instance._cancel_callback(goal_handle)

        # Verify it returns the expected response
        assert result == CancelResponse.ACCEPT
