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

ROS2_INTERFACE_PACKAGE = "niryo_ned_ros2_interfaces"
ROS1_INTERFACE_PACKAGES = [
    "niryo_robot_arm_commander",
    "niryo_robot_tools_commander",
    "niryo_robot_vision",
    "niryo_robot_user_interface",
    "niryo_robot_blockly",
    "niryo_robot_database",
    "niryo_robot_led_ring",
    "niryo_robot_poses_handlers",
    "niryo_robot_metrics",
    "niryo_robot_msgs",
    "niryo_robot_programs_manager_v2",
    "niryo_robot_programs_manager",
    "niryo_robot_rpi",
    "niryo_robot_sound",
    "niryo_robot_status",
    "niryo_robot_reports",
    "end_effector_interface",
    "joints_interface",
    "tools_interface",
    "conveyor_interface",
    "ttl_driver",
]
BLACKLISTED_INTERFACES = [
    "/niryo_robot_programs_manager/.*",
    "/connected_clients",
    "/niryo_robot_follow_joint_trajectory_controller/state",  # TODO(Thomas): we might need to find a solution to reabilitate this topic
    "/rosapi/.*",
    "/republish_tfs",
    "/foxglove_nodelet_manager/.*",
    ".*/get_loggers",
    ".*/set_logger_level",
    "/move_group/display_contacts",
    "/clear_octomap",
    "/controller_manager/.*",
    "/rviz.*",
]
LATCHED_ROS1_TOPICS = {
    "/niryo_robot_status/robot_status",
    "/niryo_robot_led_ring/led_ring_status",
    "/niryo_robot/max_velocity_scaling_factor",
    "/niryo_robot/max_acceleration_scaling_factor",
    "/niryo_robot_arm_commander/trajectory_list",
    "/niryo_robot_poses_handlers/dynamic_frame_list",
    "/niryo_robot_poses_handlers/workspace_list",
    "/niryo_robot_poses_handlers/pose_list",
    "/visualization_marker_array",
    "/niryo_robot_programs_manager_v2/program_list",
    "/niryo_robot_rpi/digital_io_state",
    "/niryo_robot_rpi/analog_io_state",
    "/niryo_robot_rpi/pause_state",
    "/niryo_robot/rpi/is_button_pressed",
    "/niryo_robot/rpi/led_state",
    "/niryo_robot/rpi/is_button_pressed",
    "/niryo_robot_sound/sound_database",
    "/niryo_robot_sound/sound",
    "/niryo_robot_sound/volume",
    "/niryo_robot_tools_commander/current_id",
    "/niryo_robot_tools_commander/tcp",
    "/niryo_robot_vision/visualization_marker",
    "/niryo_robot_vision/camera_intrinsics",
    "/niryo_robot_vision/video_stream_parameters",
}
ROS1_ACTIONS = [
    "/niryo_robot_arm_commander/robot_action",
    "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory",
    "/niryo_robot_programs_manager_v2/execute_program",
    "/niryo_robot_tools_commander/action_server",
]
INCOMPATIBLE_TYPES = [
    "dynamic_reconfigure",
    "rosgraph_msgs",
    "bond",
    "roscpp",
    "moveit_msgs",
]
