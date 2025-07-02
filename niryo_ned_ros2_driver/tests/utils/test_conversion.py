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

from niryo_ned_ros2_driver.utils.conversion import (
    ros2_message_to_dict,
    convert_ROS1_header_to_ROS2,
    convert_ROS1_time_to_ROS2,
    convert_ROS1_duration_to_ROS2,
    convert_ROS1_camera_info_to_ROS2,
    normalize_ROS1_type_to_ROS2,
    convert_ros2_time_to_ros1,
    convert_ros2_duration_to_ros1,
    convert_ros2_header_to_ros1,
    convert_ROS2_Follow_joint_traj_goal_to_ROS1,
    normalize_ROS2_type_to_ROS1,
    recursive_ros1_fields_to_ros2_normalization,
    recursive_ros2_fields_to_ros1_normalization,
)
from unittest.mock import MagicMock
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from unittest.mock import patch

class TestConversionUtils:
    """Test suite for ROS message conversion utilities."""

    def test_ros2_message_to_dict(self):
        """Test conversion of complex ROS2 messages to dictionary."""
        # Create a mock ROS2 message
        mock_msg = MagicMock()
        mock_msg.get_fields_and_field_types.return_value = {
            "header": "std_msgs/Header",
            "name": "string",
            "position": "float64",
            "velocity": "float64",
            "effort": "float64",
        }
        mock_msg.SLOT_TYPES = ["Header", "string", "float64", "float64", "float64"]

        # Set attribute values
        mock_header = MagicMock()
        mock_header.get_fields_and_field_types.return_value = {
            "stamp": "builtin_interfaces/Time",
            "frame_id": "string",
        }
        mock_header.SLOT_TYPES = ["Time", "string"]

        mock_time = MagicMock()
        mock_time.get_fields_and_field_types.return_value = {
            "sec": "int32",
            "nanosec": "uint32",
        }
        mock_time.SLOT_TYPES = ["int32", "uint32"]
        mock_time.sec = 10
        mock_time.nanosec = 500000000

        mock_header.stamp = mock_time
        mock_header.frame_id = "base_link"

        mock_msg.header = mock_header
        mock_msg.name = "joint1"
        mock_msg.position = 1.5
        mock_msg.velocity = 0.5
        mock_msg.effort = 10.0

        # Convert to dict
        result = ros2_message_to_dict(mock_msg)

        # Verify the result
        assert isinstance(result, dict)
        assert "header" in result
        assert "name" in result
        assert "position" in result
        assert result["name"] == "joint1"
        assert result["position"] == 1.5
        assert result["header"]["stamp"]["sec"] == 10
        assert result["header"]["stamp"]["nanosec"] == 500000000
        assert result["header"]["frame_id"] == "base_link"

    def test_ros2_message_to_dict_with_real_message(self):
        """Test conversion of a real complex ROS2 message to dictionary."""

        # Create a real ROS2 message
        header = Header()
        header.stamp = Time(sec=10, nanosec=500000000)
        header.frame_id = "base_link"

        joint_state_msg = JointState()
        joint_state_msg.header = header
        joint_state_msg.name = ["joint1", "joint2"]
        joint_state_msg.position = [1.5, 2.5]
        joint_state_msg.velocity = [0.5, 0.6]
        joint_state_msg.effort = [10.0, 20.0]

        # Convert to dict
        result = ros2_message_to_dict(joint_state_msg)

        # Verify the result
        assert isinstance(result, dict)
        assert "header" in result
        assert "name" in result
        assert "position" in result
        assert result["name"] == ["joint1", "joint2"]
        assert result["position"] == [1.5, 2.5]
        assert result["header"]["stamp"]["sec"] == 10
        assert result["header"]["stamp"]["nanosec"] == 500000000
        assert result["header"]["frame_id"] == "base_link"

    def test_convert_ROS1_time_to_ROS2(self):
        """Test conversion of ROS1 time to ROS2 time."""
        # Full time specification
        ros1_time = {"secs": 10, "nsecs": 500000000}
        ros2_time = convert_ROS1_time_to_ROS2(ros1_time)
        assert ros2_time["sec"] == 10
        assert ros2_time["nanosec"] == 500000000

    def test_convert_ROS1_duration_to_ROS2(self):
        """Test conversion of ROS1 duration to ROS2 duration."""
        # Full duration specification
        ros1_duration = {"secs": 5, "nsecs": 250000000}
        ros2_duration = convert_ROS1_duration_to_ROS2(ros1_duration)
        assert ros2_duration["sec"] == 5
        assert ros2_duration["nanosec"] == 250000000

    def test_convert_ROS1_header_to_ROS2(self):
        """Test conversion of ROS1 header to ROS2 header."""
        # Full header
        ros1_header = {
            "stamp": {"secs": 15, "nsecs": 750000000},
            "frame_id": "map",
            "seq": 42,
        }
        ros2_header = convert_ROS1_header_to_ROS2(ros1_header)
        assert ros2_header["stamp"]["sec"] == 15
        assert ros2_header["stamp"]["nanosec"] == 750000000
        assert ros2_header["frame_id"] == "map"
        assert "seq" not in ros2_header

    def test_convert_ROS1_camera_info_to_ROS2(self):
        """Test conversion of ROS1 camera info to ROS2 camera info."""
        # Create camera info with uppercase matrices
        ros1_camera_info = {
            "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "camera_link"},
            "D": [0.1, 0.2, 0.0, 0.0, 0.0],
            "K": [100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0],
            "R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "P": [100.0, 0.0, 320.0, 0.0, 0.0, 100.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        }

        # Convert to ROS2
        convert_ROS1_camera_info_to_ROS2(ros1_camera_info)

        # Check that matrices were renamed to lowercase
        assert "D" not in ros1_camera_info
        assert "K" not in ros1_camera_info
        assert "R" not in ros1_camera_info
        assert "P" not in ros1_camera_info

        assert "d" in ros1_camera_info
        assert "k" in ros1_camera_info
        assert "r" in ros1_camera_info
        assert "p" in ros1_camera_info

        # Check that the values were preserved
        assert ros1_camera_info["d"] == [0.1, 0.2, 0.0, 0.0, 0.0]
        assert ros1_camera_info["k"] == [
            100.0,
            0.0,
            320.0,
            0.0,
            100.0,
            240.0,
            0.0,
            0.0,
            1.0,
        ]
        assert ros1_camera_info["r"] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        assert ros1_camera_info["p"] == [
            100.0,
            0.0,
            320.0,
            0.0,
            0.0,
            100.0,
            240.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]


    def test_normalize_ROS1_type_to_ROS2(self):
        """Test normalization of ROS1 message to ROS2 by type."""
        # Test with marker array
        ros1_marker_array = {
            "markers": [
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "map"},
                    "lifetime": {"secs": 5, "nsecs": 0},
                }
            ]
        }

        from visualization_msgs.msg import MarkerArray
        field_types = MarkerArray().get_fields_and_field_types()
        normalize_ROS1_type_to_ROS2(ros1_marker_array, field_types)

        # Check the result
        assert ros1_marker_array["markers"][0]["header"]["stamp"]["sec"] == 10
        assert ros1_marker_array["markers"][0]["header"]["stamp"]["nanosec"] == 0
        assert ros1_marker_array["markers"][0]["lifetime"]["sec"] == 5
        assert ros1_marker_array["markers"][0]["lifetime"]["nanosec"] == 0

        # Test with camera info
        ros1_camera_info = {
            "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "camera_link"},
            "D": [0.1, 0.2, 0.0, 0.0, 0.0],
            "K": [100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0],
        }

        from sensor_msgs.msg import CameraInfo
        field_types = CameraInfo().get_fields_and_field_types()
        normalize_ROS1_type_to_ROS2(ros1_camera_info, field_types)

        # Check the result
        assert ros1_camera_info["header"]["stamp"]["sec"] == 10
        assert ros1_camera_info["header"]["stamp"]["nanosec"] == 0
        assert "d" in ros1_camera_info
        assert "k" in ros1_camera_info

    def test_convert_ros2_time_to_ros1(self):
        """Test conversion of ROS2 time to ROS1 time."""
        # Full time specification
        ros2_time = {"sec": 10, "nanosec": 500000000}
        ros1_time = convert_ros2_time_to_ros1(ros2_time)
        assert ros1_time["secs"] == 10
        assert ros1_time["nsecs"] == 500000000

    def test_convert_ros2_duration_to_ros1(self):
        """Test conversion of ROS2 duration to ROS1 duration."""
        # Full duration specification
        ros2_duration = {"sec": 5, "nanosec": 250000000}
        ros1_duration = convert_ros2_duration_to_ros1(ros2_duration)
        assert ros1_duration["secs"] == 5
        assert ros1_duration["nsecs"] == 250000000

    def test_convert_ros2_header_to_ros1(self):
        """Test conversion of ROS2 header to ROS1 header."""
        # Full header
        ros2_header = {"stamp": {"sec": 15, "nanosec": 750000000}, "frame_id": "map"}
        ros1_header = convert_ros2_header_to_ros1(ros2_header)
        assert ros1_header["stamp"]["secs"] == 15
        assert ros1_header["stamp"]["nsecs"] == 750000000
        assert ros1_header["frame_id"] == "map"

    def test_convert_ROS2_Follow_joint_traj_goal_to_ROS1(self):
        """Test conversion of ROS2 FollowJointTrajectory goal to ROS1."""
        # Create a sample goal with fields that should be removed
        ros2_goal = {
            "trajectory": {
                "header": {"stamp": {"sec": 10, "nanosec": 0}, "frame_id": "base_link"},
                "joint_names": ["joint1", "joint2"],
                "points": [
                    {
                        "positions": [0.0, 0.0],
                        "time_from_start": {"sec": 1, "nanosec": 0},
                    }
                ],
            },
            "multi_dof_trajectory": {},
            "component_path_tolerance": [],
            "component_goal_tolerance": [],
            "goal_time_tolerance": {"sec": 1, "nanosec": 0},
        }

        # Convert to ROS1
        convert_ROS2_Follow_joint_traj_goal_to_ROS1(ros2_goal)

        # Check that fields were removed
        assert "multi_dof_trajectory" not in ros2_goal
        assert "component_path_tolerance" not in ros2_goal
        assert "component_goal_tolerance" not in ros2_goal

        # Check that other fields remain
        assert "trajectory" in ros2_goal
        assert "goal_time_tolerance" in ros2_goal

    def test_recursive_ros2_fields_to_ros1_normalization(self):
        """Test recursive normalization of ROS2 fields to ROS1 using control_msgs/action/FollowJointTrajectory."""
        # Create a ROS2 FollowJointTrajectory message
        ros2_msg = {
            "trajectory": {
                "header": {
                    "stamp": {"sec": 10, "nanosec": 500000000},
                    "frame_id": "map",
                },
                "joint_names": ["joint1", "joint2"],
                "points": [
                    {
                        "positions": [0.0, 1.0],
                        "velocities": [0.1, 0.2],
                        "accelerations": [0.01, 0.02],
                        "effort": [0.5, 0.6],
                        "time_from_start": {"sec": 5, "nanosec": 0},
                    },
                    {
                        "positions": [1.0, 2.0],
                        "velocities": [0.2, 0.3],
                        "accelerations": [0.02, 0.03],
                        "effort": [0.6, 0.7],
                        "time_from_start": {"sec": 10, "nanosec": 0},
                    },
                ],
            },
            "goal_time_tolerance": {"sec": 1, "nanosec": 0},
        }

        # Apply recursive normalization
        recursive_ros2_fields_to_ros1_normalization(ros2_msg)

        # Check the results
        assert ros2_msg["trajectory"]["header"]["stamp"]["secs"] == 10
        assert ros2_msg["trajectory"]["header"]["stamp"]["nsecs"] == 500000000

        assert ros2_msg["trajectory"]["points"][0]["time_from_start"]["secs"] == 5
        assert ros2_msg["trajectory"]["points"][0]["time_from_start"]["nsecs"] == 0

        assert ros2_msg["trajectory"]["points"][1]["time_from_start"]["secs"] == 10
        assert ros2_msg["trajectory"]["points"][1]["time_from_start"]["nsecs"] == 0

        assert ros2_msg["goal_time_tolerance"]["secs"] == 1
        assert ros2_msg["goal_time_tolerance"]["nsecs"] == 0

    def test_normalize_ROS2_type_to_ROS1(self):
        """Test normalization of ROS2 message to ROS1 by type."""
        # Test with FollowJointTrajectoryAction goal
        ros2_goal = {
            "trajectory": {},
            "multi_dof_trajectory": {},
            "goal_time_tolerance": {"sec": 1, "nanosec": 0},
        }

        normalize_ROS2_type_to_ROS1(
            ros2_goal, "control_msgs/FollowJointTrajectoryAction"
        )

        # Check the result
        assert "multi_dof_trajectory" not in ros2_goal
        assert "trajectory" in ros2_goal

        # Test with a message containing headers and time fields
        ros2_msg = {
            "header": {"stamp": {"sec": 10, "nanosec": 0}, "frame_id": "base_link"},
            "time_from_start": {"sec": 5, "nanosec": 0},
        }

        normalize_ROS2_type_to_ROS1(ros2_msg, "any_type_with_header")

        # Check the result
        assert ros2_msg["header"]["stamp"]["secs"] == 10
        assert ros2_msg["header"]["stamp"]["nsecs"] == 0
        assert ros2_msg["time_from_start"]["secs"] == 5
        assert ros2_msg["time_from_start"]["nsecs"] == 0

        # Test with a message containing nested fields
        ros2_nested_msg = {
            "outer_field": {
                "inner_field": {
                    "header": {"stamp": {"sec": 20, "nanosec": 500000000}},
                    "time_from_start": {"sec": 2, "nanosec": 100000000},
                }
            }
        }

        normalize_ROS2_type_to_ROS1(ros2_nested_msg, "nested_type")

        # Check the result
        assert (
            ros2_nested_msg["outer_field"]["inner_field"]["header"]["stamp"]["secs"]
            == 20
        )
        assert (
            ros2_nested_msg["outer_field"]["inner_field"]["header"]["stamp"]["nsecs"]
            == 500000000
        )
        assert (
            ros2_nested_msg["outer_field"]["inner_field"]["time_from_start"]["secs"]
            == 2
        )
        assert (
            ros2_nested_msg["outer_field"]["inner_field"]["time_from_start"]["nsecs"]
            == 100000000
        )


    def test_recursive_ros1_fields_to_ros2_normalization(self):
        """Test recursive normalization with explicit type information."""
        
        # Test with header field conversion
        obj = {
            "header": {
                "stamp": {"secs": 10, "nsecs": 500000000},
                "frame_id": "base_link"
            },
            "other_field": "value"
        }
        field_types = {
            "header": "std_msgs/Header",
            "other_field": "string"
        }
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        assert obj["header"]["stamp"]["sec"] == 10
        assert obj["header"]["stamp"]["nanosec"] == 500000000
        assert obj["header"]["frame_id"] == "base_link"
        assert obj["other_field"] == "value"

    def test_recursive_ros1_fields_to_ros2_normalization_camera_info(self):
        """Test recursive normalization with camera info type conversion."""
        
        obj = {
            "camera_info": {
                "D": [0.1, 0.2],
                "K": [100.0, 0.0, 320.0],
                "header": {
                    "stamp": {"secs": 5, "nsecs": 0},
                    "frame_id": "camera"
                }
            }
        }
        field_types = {
            "camera_info": "sensor_msgs/CameraInfo"
        }
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Check type-specific conversion (D->d, K->k)
        assert "d" in obj["camera_info"]
        assert "k" in obj["camera_info"]
        assert "D" not in obj["camera_info"]
        assert "K" not in obj["camera_info"]
        # Check field-based conversion for header
        assert obj["camera_info"]["header"]["stamp"]["sec"] == 5
        assert obj["camera_info"]["header"]["stamp"]["nanosec"] == 0

    def test_recursive_ros1_fields_to_ros2_normalization_nested_messages(self):
        """Test recursive normalization with nested message types."""
        
        obj = {
            "trajectory": {
                "header": {
                    "stamp": {"secs": 15, "nsecs": 750000000},
                    "frame_id": "map"
                },
                "points": [
                    {
                        "time_from_start": {"secs": 2, "nsecs": 0}
                    }
                ]
            }
        }
        field_types = {
            "trajectory": "trajectory_msgs/JointTrajectory"
        }
        
        # Mock get_nested_field_types to return nested field types
        with patch('niryo_ned_ros2_driver.utils.conversion.get_nested_field_types') as mock_get_types:
            mock_get_types.return_value = {
                "header": "std_msgs/Header",
                "points": "trajectory_msgs/JointTrajectoryPoint[]"
            }
            
            recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Check that nested header was converted
        assert obj["trajectory"]["header"]["stamp"]["sec"] == 15
        assert obj["trajectory"]["header"]["stamp"]["nanosec"] == 750000000
        # Check that nested time_from_start was converted
        assert obj["trajectory"]["points"][0]["time_from_start"]["sec"] == 2
        assert obj["trajectory"]["points"][0]["time_from_start"]["nanosec"] == 0

    def test_recursive_ros1_fields_to_ros2_normalization_primitive_types(self):
        """Test recursive normalization with primitive types."""
        
        obj = {
            "name": "test",
            "value": 42,
            "active": True,
            "nested": {
                "stamp": {"secs": 1, "nsecs": 0}
            }
        }
        field_types = {
            "name": "string",
            "value": "int32",
            "active": "bool",
            "nested": "some_primitive_type"
        }
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Primitive types should remain unchanged
        assert obj["name"] == "test"
        assert obj["value"] == 42
        assert obj["active"] == True
        # Nested stamp should be converted
        assert obj["nested"]["stamp"]["sec"] == 1
        assert obj["nested"]["stamp"]["nanosec"] == 0

    def test_recursive_ros1_fields_to_ros2_normalization_list_processing(self):
        """Test recursive normalization with lists."""
        
        obj = [
            {
                "header": {
                    "stamp": {"secs": 20, "nsecs": 0},
                    "frame_id": "frame1"
                }
            },
            {
                "header": {
                    "stamp": {"secs": 30, "nsecs": 500000000},
                    "frame_id": "frame2"
                }
            }
        ]
        field_types = {}
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Check that all items in the list were processed
        assert obj[0]["header"]["stamp"]["sec"] == 20
        assert obj[0]["header"]["stamp"]["nanosec"] == 0
        assert obj[1]["header"]["stamp"]["sec"] == 30
        assert obj[1]["header"]["stamp"]["nanosec"] == 500000000

    def test_recursive_ros1_fields_to_ros2_normalization_empty_field_types(self):
        """Test recursive normalization with empty field types."""
        
        obj = {
            "header": {
                "stamp": {"secs": 25, "nsecs": 0},
                "frame_id": "test"
            },
            "time_from_start": {"secs": 3, "nsecs": 250000000}
        }
        field_types = {}
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Should still apply field-based conversions even without type info
        assert obj["header"]["stamp"]["sec"] == 25
        assert obj["header"]["stamp"]["nanosec"] == 0
        assert obj["time_from_start"]["sec"] == 3
        assert obj["time_from_start"]["nanosec"] == 250000000

    def test_recursive_ros1_fields_to_ros2_normalization_non_dict_values(self):
        """Test recursive normalization with non-dictionary values."""
        
        obj = {
            "string_field": "test_value",
            "number_field": 123,
            "list_field": [1, 2, 3],
            "nested_dict": {
                "stamp": {"secs": 5, "nsecs": 0}
            }
        }
        field_types = {
            "string_field": "string",
            "number_field": "int32",
            "list_field": "int32[]",
            "nested_dict": "custom_type"
        }
        
        recursive_ros1_fields_to_ros2_normalization(obj, field_types)
        
        # Non-dict values should remain unchanged except for nested conversions
        assert obj["string_field"] == "test_value"
        assert obj["number_field"] == 123
        assert obj["list_field"] == [1, 2, 3]
        assert obj["nested_dict"]["stamp"]["sec"] == 5
        assert obj["nested_dict"]["stamp"]["nanosec"] == 0

