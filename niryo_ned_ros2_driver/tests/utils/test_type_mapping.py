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
from unittest.mock import patch
from niryo_ned_ros2_driver.utils.type_mapping import (
    convert_ros1_to_ros2_type,
    guess_action_type_from_goal_type,
)


class TestTypeMapping:
    """Tests for the type mapping utility functions."""

    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS1_INTERFACE_PACKAGES",
        ["niryo_robot_msgs"],
    )
    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS2_INTERFACE_PACKAGE",
        "niryo_ned_ros2_interfaces",
    )
    def test_convert_ros1_to_ros2_type_message(self):
        """Test conversion of ROS1 message types to ROS2 format."""
        # Test standard message conversion (not in ROS1_INTERFACE_PACKAGES)
        ros2_type = convert_ros1_to_ros2_type("geometry_msgs/Point", "msg")
        assert ros2_type == "geometry_msgs/msg/Point"

        # Test message in ROS1_INTERFACE_PACKAGES (should change package name)
        ros2_type = convert_ros1_to_ros2_type("niryo_robot_msgs/RobotStatus", "msg")
        assert ros2_type == "niryo_ned_ros2_interfaces/msg/RobotStatus"

    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS1_INTERFACE_PACKAGES",
        ["ttl_driver"],
    )
    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS2_INTERFACE_PACKAGE",
        "niryo_ned_ros2_interfaces",
    )
    def test_convert_ros1_to_ros2_type_service(self):
        """Test conversion of ROS1 service types to ROS2 format."""
        # Test standard service conversion (not in ROS1_INTERFACE_PACKAGES)
        ros2_type = convert_ros1_to_ros2_type("std_srvs/Trigger", "srv")
        assert ros2_type == "std_srvs/srv/Trigger"

        # Test service in ROS1_INTERFACE_PACKAGES (should change package name)
        ros2_type = convert_ros1_to_ros2_type("ttl_driver/WritePIDValue", "srv")
        assert ros2_type == "niryo_ned_ros2_interfaces/srv/WritePIDValue"

    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS1_INTERFACE_PACKAGES",
        ["niryo_robot_arm_commander"],
    )
    @patch(
        "niryo_ned_ros2_driver.utils.type_mapping.ROS2_INTERFACE_PACKAGE",
        "niryo_ned_ros2_interfaces",
    )
    def test_convert_ros1_to_ros2_type_action(self):
        """Test conversion of ROS1 action types to ROS2 format."""
        # Test standard action conversion (not in ROS1_INTERFACE_PACKAGES)
        ros2_type = convert_ros1_to_ros2_type(
            "control_msgs/FollowJointTrajectoryAction", "action"
        )
        assert (
            ros2_type == "control_msgs/action/FollowJointTrajectory"
        )  # Note: "Action" is removed

        # Test action in ROS1_INTERFACE_PACKAGES (should change package name and remove "Action")
        ros2_type = convert_ros1_to_ros2_type(
            "niryo_robot_arm_commander/RobotMoveAction", "action"
        )
        assert ros2_type == "niryo_ned_ros2_interfaces/action/RobotMove"

    def test_convert_ros1_to_ros2_type_invalid_interface(self):
        """Test conversion with an invalid interface type."""
        with pytest.raises(ValueError) as excinfo:
            convert_ros1_to_ros2_type("std_msgs/String", "invalid_type")

        assert "Invalid interface type" in str(excinfo.value)
        assert "Expected 'srv', 'msg' or 'action'" in str(excinfo.value)

    def test_convert_ros1_to_ros2_type_invalid_format(self):
        """Test conversion with an invalid type format."""
        with pytest.raises(ValueError):
            convert_ros1_to_ros2_type("invalid_format", "msg")

    def test_guess_action_type_from_goal_type(self):
        """Test guessing the action type from a goal type."""
        # Test with standard action goal
        action_type = guess_action_type_from_goal_type(
            "control_msgs/FollowJointTrajectoryActionGoal"
        )
        assert action_type == "control_msgs/FollowJointTrajectoryAction"

    def test_guess_action_type_from_goal_type_invalid(self):
        """Test guessing the action type from an invalid goal type."""
        # Test with non-goal type
        with pytest.raises(ValueError) as excinfo:
            guess_action_type_from_goal_type("control_msgs/FollowJointTrajectory")

        assert "is not an ActionGoal type" in str(excinfo.value)

        # Test with malformed type
        with pytest.raises(ValueError):
            guess_action_type_from_goal_type("invalid_format")
