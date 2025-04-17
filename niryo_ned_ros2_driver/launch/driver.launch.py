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
    use_whitelist = LaunchConfiguration("use_whitelist")
    log_level = LaunchConfiguration("log_level")
    drivers_list_filepath = LaunchConfiguration("drivers_list_filepath").perform(
        context
    )
    whitelist_filepath = LaunchConfiguration("whitelist_filepath").perform(context)

    with open(drivers_list_filepath, "r") as f:
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

    with open(whitelist_filepath, "r") as f:
        driver_whitelists = yaml.safe_load(f)

    whitelist_topics = driver_whitelists.get("whitelist_topics", {})
    whitelist_services = driver_whitelists.get("whitelist_services", {})

    driver_nodes = []

    for ip, ns in zip(robot_ips, robot_namespaces):
        driver_node = Node(
            package="niryo_ned_ros2_driver",
            executable="ros2_driver",
            name=f"ros2_driver_{ns if ns else 'default'}",
            parameters=[
                {
                    "rosbridge_port": rosbridge_port,
                    "robot_ip": ip,
                    "robot_namespace": ns,
                    "use_whitelist": use_whitelist,
                    "whitelist_topics": whitelist_topics,
                    "whitelist_services": whitelist_services,
                },
            ],
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

    default_whitelist_filepath = os.path.join(
        get_package_share_directory("niryo_ned_ros2_driver"),
        "config",
        "whitelist.yaml",
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "drivers_list_filepath",
            default_value=drivers_list_filepath,
            description="Path to the drivers list file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_whitelist",
            default_value="false",
            description="Whether only a subset of interfaces should be bridged",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "whitelist_filepath",
            default_value=default_whitelist_filepath,
            description="Path to the whitelist file",
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
