# /usr/bin/env python3

import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
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

        topic_whitelist = (
            self.get_parameter("topic_whitelist")
            .get_parameter_value()
            .string_array_value
        )
        service_whitelist = (
            self.get_parameter("service_whitelist")
            .get_parameter_value()
            .string_array_value
        )
        action_whitelist = (
            self.get_parameter("action_whitelist")
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
                topic_whitelist=topic_whitelist,
                service_whitelist=service_whitelist,
                action_whitelist=action_whitelist,
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
            "topic_whitelist",
            [".*"],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of regex patterns for whitelisted topics",
            ),
        )

        self.declare_parameter(
            "service_whitelist",
            [".*"],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of regex patterns for whitelisted services",
            ),
        )

        self.declare_parameter(
            "action_whitelist",
            [".*"],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of regex patterns for whitelisted actions",
            ),
        )


def main():
    rclpy.init()

    node = Bridge()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting...")
    except Exception as e:
        node.get_logger().error(f"Exception in bridge main loop: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
