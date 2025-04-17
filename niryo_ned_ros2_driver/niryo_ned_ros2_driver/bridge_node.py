# /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .driver import ROS2Driver


class Bridge(Node):
    def __init__(self):
        super().__init__("ros2_bridge_node")

        self._declare_parameters()

        namespace = (
            self.get_parameter("robot_namespace").get_parameter_value().string_value
        )
        ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        port = self.get_parameter("rosbridge_port").get_parameter_value().integer_value

        if not ip:
            self.get_logger().error("Robot IP is required")
            rclpy.shutdown()
            sys.exit(1)

        use_whitelist = (
            self.get_parameter("use_whitelist").get_parameter_value().bool_value
        )

        whitelist_topics = []
        whitelist_services = []
        if use_whitelist:
            whitelist_topics = (
                self.get_parameter("whitelist_topics")
                .get_parameter_value()
                .string_array_value
            )
            whitelist_services = (
                self.get_parameter("whitelist_services")
                .get_parameter_value()
                .string_array_value
            )

        self.get_logger().info(
            f"Creating driver for robot with IP: {ip} and port: {port}"
        )

        try:
            self._driver = ROS2Driver(
                self,
                namespace,
                ip,
                port,
                use_whitelist,
                whitelist_topics,
                whitelist_services,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create driver: {e}")
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(f"Bridge node initialized for robot with ip: {ip}")

    def _declare_parameters(self):
        self.declare_parameter(
            "rosbridge_port",
            9090,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Port for the ROSBridge server",
            ),
        )
        self.declare_parameter(
            "robot_namespace",
            "",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Robot's namespace",
            ),
        )
        self.declare_parameter(
            "robot_ip",
            "",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Robot's IP address",
            ),
        )

        self.declare_parameter(
            "use_whitelist",
            False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether only a subset of interfaces should be bridged",
            ),
        )

        self.declare_parameter(
            "whitelist_topics",
            [""],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of whitelisted topics if use_whitelist is True",
            ),
        )

        self.declare_parameter(
            "whitelist_services",
            [""],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of whitelisted services if use_whitelist is True",
            ),
        )


def main():
    rclpy.init()

    node = Bridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
