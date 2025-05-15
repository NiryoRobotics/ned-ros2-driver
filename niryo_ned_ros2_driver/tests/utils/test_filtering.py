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

from unittest.mock import patch
from niryo_ned_ros2_driver.utils.filtering import (
    compile_regex_list,
    matches_any,
    is_whitelisted,
    is_blacklisted,
    is_action_topic,
    is_non_existing_ros2_type,
    filter_topics,
    filter_services,
    filter_actions,
)


class TestFiltering:
    """Test suite for the filtering utilities."""

    def test_matches_any(self):
        """Test matching a string against a list of patterns."""
        # Compile some test patterns
        patterns = compile_regex_list(["^/test/.*$", "^/robot/.*$", "^/global$"])

        # Test matches
        assert matches_any("/test/topic", patterns)
        assert matches_any("/robot/arm/position", patterns)
        assert matches_any("/global", patterns)

        # Test non-matches
        assert not matches_any("/other/topic", patterns)
        assert not matches_any("global", patterns)  # Missing leading slash
        assert not matches_any("/test", patterns)  # Missing trailing content

    def test_is_whitelisted(self):
        """Test the whitelisting functionality."""
        # Compile whitelist patterns
        patterns = compile_regex_list(["^/allowed/.*$", "^/special$"])

        # Test whitelisted topics
        assert is_whitelisted("/allowed/topic", patterns)
        assert is_whitelisted("/allowed/subtopic/value", patterns)
        assert is_whitelisted("/special", patterns)

        # Test non-whitelisted topics
        assert not is_whitelisted("/notallowed/topic", patterns)
        assert not is_whitelisted("/special/subtopic", patterns)

    def test_is_blacklisted(self):
        """Test the blacklisting functionality."""
        with patch("niryo_ned_ros2_driver.utils.filtering.matches_any") as mock_matches:
            # Test blacklisted
            mock_matches.return_value = True
            assert is_blacklisted("/some/topic")

            # Test not blacklisted
            mock_matches.return_value = False
            assert not is_blacklisted("/public/topic")

    def test_is_action_topic(self):
        """Test detection of action-related topics."""
        # Test action topics
        action_topics = [
            "/move_robot/goal",
            "/arm_controller/follow_joint_trajectory/cancel",
            "/some_action/status",
            "/gripper_action/result",
            "/trajectory/feedback",
        ]
        for topic in action_topics:
            assert is_action_topic(topic)

        # Test non-action topics
        non_action_topics = [
            "/joint_states",
            "/cancel/action",  # Suffix in wrong position
        ]
        for topic in non_action_topics:
            assert not is_action_topic(topic)

    @patch("niryo_ned_ros2_driver.utils.filtering.INCOMPATIBLE_TYPES")
    def test_is_non_existing_ros2_type(self, mock_incompatible):
        """Test detection of incompatible ROS2 types."""
        # Set up mock incompatible types
        mock_incompatible.__iter__.return_value = ["outdated_msgs/", "legacy_srvs/"]

        # Test incompatible types
        assert is_non_existing_ros2_type("outdated_msgs/OldType")
        assert is_non_existing_ros2_type("legacy_srvs/LegacyService")

        # Test compatible types
        assert not is_non_existing_ros2_type("std_msgs/String")
        assert not is_non_existing_ros2_type("geometry_msgs/Pose")
        assert not is_non_existing_ros2_type("sensor_msgs/Image")

    def test_filter_topics(self):
        """Test filtering of topics based on whitelist and blacklist."""
        # Set up a mock topic_type_map
        topic_type_map = {
            "/joint_states": "sensor_msgs/JointState",
            "/tf": "tf2_msgs/TFMessage",
            "/robot/goal": "control_msgs/FollowJointTrajectoryActionGoal",  # Action topic
            "/private/data": "std_msgs/String",  # Blacklisted
            "/old_topic": "outdated_msgs/OldType",  # Incompatible type
        }

        # Set up whitelist patterns
        whitelist = [
            "^/joint_states$",
            "^/tf$",
            "^/robot/.*$",
            "^/private/.*$",
        ]

        # Mock the required functions
        with patch(
            "niryo_ned_ros2_driver.utils.filtering.is_action_topic"
        ) as mock_is_action:
            with patch(
                "niryo_ned_ros2_driver.utils.filtering.is_non_existing_ros2_type"
            ) as mock_incompatible:
                with patch(
                    "niryo_ned_ros2_driver.utils.filtering.is_blacklisted"
                ) as mock_blacklisted:
                    # Set up return values for each topic
                    def is_action_side_effect(topic):
                        return topic == "/robot/goal"

                    def is_incompatible_side_effect(type_name):
                        return type_name == "outdated_msgs/OldType"

                    def is_blacklisted_side_effect(topic):
                        return topic == "/private/data"

                    mock_is_action.side_effect = is_action_side_effect
                    mock_incompatible.side_effect = is_incompatible_side_effect
                    mock_blacklisted.side_effect = is_blacklisted_side_effect

                    result = filter_topics(topic_type_map, whitelist)

                    # Check the results
                    assert "/joint_states" in result
                    assert "/tf" in result
                    assert (
                        "/robot/goal" not in result
                    )  # Filtered because it's an action topic
                    assert (
                        "/private/data" not in result
                    )  # Filtered because it's blacklisted
                    assert (
                        "/old_topic" not in result
                    )  # Filtered because type is incompatible

                    # Check that the filtered map has the right number of entries
                    assert len(result) == 2

    def test_filter_services(self):
        """Test filtering of services based on whitelist and blacklist."""
        # Set up a mock service_type_map
        service_type_map = {
            "/get_joint_states": "niryo_robot_msgs/GetJointStates",
            "/set_pose": "niryo_robot_msgs/SetPose",
            "/internal/calibrate": "std_srvs/Trigger",  # Blacklisted
            "/legacy_service": "legacy_srvs/LegacyService",  # Incompatible type
        }

        # Set up whitelist patterns
        whitelist = ["^/get_.*$", "^/set_.*$", "^/internal/.*$"]

        # Mock the required functions
        with patch(
            "niryo_ned_ros2_driver.utils.filtering.is_non_existing_ros2_type"
        ) as mock_incompatible:
            with patch(
                "niryo_ned_ros2_driver.utils.filtering.is_blacklisted"
            ) as mock_blacklisted:
                # Set up return values for each service
                def is_incompatible_side_effect(type_name):
                    return type_name == "legacy_srvs/LegacyService"

                def is_blacklisted_side_effect(service):
                    return service == "/internal/calibrate"

                mock_incompatible.side_effect = is_incompatible_side_effect
                mock_blacklisted.side_effect = is_blacklisted_side_effect

                # Run the filter_services function
                result = filter_services(service_type_map, whitelist)

                # Check the results
                assert "/get_joint_states" in result
                assert "/set_pose" in result
                assert (
                    "/internal/calibrate" not in result
                )  # Filtered because it's blacklisted
                assert (
                    "/legacy_service" not in result
                )  # Filtered because type is incompatible

                # Check that the filtered map has the right number of entries
                assert len(result) == 2

    def test_filter_actions(self):
        """Test filtering of actions based on whitelist and blacklist."""
        # Set up a mock action_type_map
        action_type_map = {
            "/move_robot": "control_msgs/FollowJointTrajectory",
            "/gripper": "control_msgs/GripperCommand",
            "/internal/sequence": "niryo_robot_msgs/ExecuteSequence",  # Blacklisted
            "/legacy_action": "legacy_actions/LegacyAction",  # Incompatible type
        }

        # Set up whitelist patterns
        whitelist = ["^/move_.*$", "^/gripper$", "^/internal/.*$"]

        # Mock the required functions
        with patch(
            "niryo_ned_ros2_driver.utils.filtering.is_non_existing_ros2_type"
        ) as mock_incompatible:
            with patch(
                "niryo_ned_ros2_driver.utils.filtering.is_blacklisted"
            ) as mock_blacklisted:
                # Set up return values for each action
                def is_incompatible_side_effect(type_name):
                    return type_name == "legacy_actions/LegacyAction"

                def is_blacklisted_side_effect(action):
                    return action == "/internal/sequence"

                mock_incompatible.side_effect = is_incompatible_side_effect
                mock_blacklisted.side_effect = is_blacklisted_side_effect

                # Run the filter_actions function
                result = filter_actions(action_type_map, whitelist)

                # Check the results
                assert "/move_robot" in result
                assert "/gripper" in result
                assert (
                    "/internal/sequence" not in result
                )  # Filtered because it's blacklisted
                assert (
                    "/legacy_action" not in result
                )  # Filtered because type is incompatible

                # Check that the filtered map has the right number of entries
                assert len(result) == 2

    def test_end_to_end_topic_filtering(self):
        """Test end-to-end topic filtering with real function calls."""
        # This tests the actual function implementation without mocks

        # Set up a mock topic_type_map
        topic_type_map = {
            "/joint_states": "sensor_msgs/JointState",
            "/tf": "tf2_msgs/TFMessage",
            "/some_action/goal": "control_msgs/FollowJointTrajectoryActionGoal",
        }

        # Set up whitelist patterns
        whitelist = ["^/joint_states$", "^/some_action/.*$"]

        # We need to patch the blacklist and incompatible type checks
        # since they rely on imported constants
        with patch(
            "niryo_ned_ros2_driver.utils.filtering.is_blacklisted", return_value=False
        ):
            with patch(
                "niryo_ned_ros2_driver.utils.filtering.is_non_existing_ros2_type",
                return_value=False,
            ):
                # Run the filter_topics function
                result = filter_topics(topic_type_map, whitelist)

                # Check the results
                assert "/joint_states" in result
                assert "/tf" not in result  # Not in whitelist
                assert "/some_action/goal" not in result  # Action topic
