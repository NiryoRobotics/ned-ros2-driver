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
    "end_effector_interface",
    "joints_interface",
    "tools_interface",
    "ttl_driver",
]
BLACKLISTED_INTERFACES = [
    "/niryo_robot_programs_manager/program_list",
    "/niryo_robot_programs_manager/program_is_running",
    "/connected_clients",
    "/niryo_robot_follow_joint_trajectory_controller/state",  # TODO(Thomas): we might need to find a solution to reabilitate this topic
]
