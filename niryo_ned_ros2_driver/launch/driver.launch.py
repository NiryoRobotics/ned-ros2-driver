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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
)
import yaml
from ament_index_python.packages import get_package_share_directory
import os

from rclpy import logging

logger = logging.get_logger("ned_ros2_driver.launch")


def launch_setup(context):
    log_level = LaunchConfiguration("log_level")
    drivers_list_file = LaunchConfiguration("drivers_list_file").perform(context)
    whitelist_params_file = LaunchConfiguration("whitelist_params_file").perform(
        context
    )
    topic_whitelist = LaunchConfiguration("topic_whitelist")
    service_whitelist = LaunchConfiguration("service_whitelist")
    action_whitelist = LaunchConfiguration("action_whitelist")

    with open(drivers_list_file, "r") as f:
        drivers_list_config = yaml.safe_load(f)

    robot_ips = drivers_list_config.get("robot_ips", {})
    robot_namespaces = drivers_list_config.get("robot_namespaces", {})
    rosbridge_port = drivers_list_config.get("rosbridge_port", {})

    if len(robot_ips) != len(robot_namespaces):
        raise RuntimeError("Robot ips and robot namespaces must have the same length")

    if len(set(robot_ips)) != len(robot_ips):
        raise RuntimeError("Robot ips must be unique")
    if len(set(robot_namespaces)) != len(robot_namespaces):
        raise RuntimeError("Robot namespaces must be unique")

    driver_nodes = []

    for ip, ns in zip(robot_ips, robot_namespaces):
        params_dict = {
            "rosbridge_port": rosbridge_port,
            "robot_ip": ip,
            "robot_namespace": ns,
            "topic_whitelist": topic_whitelist,
            "service_whitelist": service_whitelist,
            "action_whitelist": action_whitelist,
        }

        params = [params_dict]
        if whitelist_params_file:
            if not os.path.isfile(whitelist_params_file):
                raise RuntimeError(
                    f"Whitelist params file not found: {whitelist_params_file}"
                )
            params.append(whitelist_params_file)

        driver_node = Node(
            package="niryo_ned_ros2_driver",
            executable="ros2_driver",
            name=f"ros2_driver_{ns if ns else 'default'}",
            parameters=params,
            arguments=["--ros-args", "--log-level", log_level],
        )

        driver_nodes.append(driver_node)

    return driver_nodes


def generate_launch_description():
    declared_arguments = []

    drivers_list_filepath = os.path.join(
        get_package_share_directory("niryo_ned_ros2_driver"),
        "config",
        "drivers_list.yaml",
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "drivers_list_file",
            default_value=drivers_list_filepath,
            description="Path to the drivers list file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "whitelist_params_file",
            default_value="",
            description="Optional path to a parameter file with whitelist parameters",
        ),
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_whitelist",
            default_value="['.*']",
            description="List of regex patterns for whitelisted topics",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "service_whitelist",
            default_value="['.*']",
            description="List of regex patterns for whitelisted services",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "action_whitelist",
            default_value="['.*']",
            description="List of regex patterns for whitelisted actions",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
            description="Logging level",
            choices=["DEBUG", "INFO", "WARN", "ERROR"],
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
