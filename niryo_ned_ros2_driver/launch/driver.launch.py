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

    bridge_list_config_path = os.path.join(
        get_package_share_directory("niryo_ned_ros2_driver"),
        "config",
        "bridge_list.yaml",
    )
    driver_config_path = os.path.join(
        get_package_share_directory("niryo_ned_ros2_driver"),
        "config",
        "driver_config.yaml",
    )

    with open(bridge_list_config_path, "r") as f:
        bridge_configs = yaml.safe_load(f)

    robot_ips = bridge_configs.get("robot_ips", {})
    robot_namespaces = bridge_configs.get("robot_namespaces", {})
    rosbridge_port = bridge_configs.get("rosbridge_port", {})

    if len(robot_ips) != len(robot_namespaces):
        raise RuntimeError("Robot ips and robot namespaces must have the same length")

    if len(set(robot_ips)) != len(robot_ips):
        raise RuntimeError("Robot ips must be unique")
    if len(set(robot_namespaces)) != len(robot_namespaces):
        raise RuntimeError("Robot namespaces must be unique")

    with open(driver_config_path, "r") as f:
        driver_configs = yaml.safe_load(f)

    whitelist_interfaces = driver_configs.get("whitelist_interfaces", {})

    driver_nodes = []

    for ip, ns in zip(robot_ips, robot_namespaces):
        driver_node = Node(
            package="niryo_ned_ros2_driver",
            executable="ros2_driver",
            name=f"ros2_driver_{ns if ns else 'default'}",
            namespace=ns,
            parameters=[
                {
                    "rosbridge_port": rosbridge_port,
                    "robot_ip": ip,
                    "robot_namespace": ns,
                    "use_whitelist": use_whitelist,
                    "whitelist_interfaces": whitelist_interfaces,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )

        driver_nodes.append(driver_node)

    return driver_nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_whitelist",
            default_value="false",
            description="Whether only a subset of interfaces should be bridged",
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
