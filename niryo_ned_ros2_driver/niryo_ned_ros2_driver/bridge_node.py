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

import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .ros2_driver import ROS2Driver


class Bridge(Node):
    """
    A ROS 2 node that acts as a bridge between a local ROS 2 system and a remote robot running ROS 1.

    This bridge node connects to a remote robot using ROSBridge protocol, providing access to the robot's
    topics, services, and actions through corresponding ROS 2 interfaces. It uses whitelists to control
    which topics, services, and actions are exposed to the local ROS 2 system.

    Parameters
    ----------
    robot_namespace : str
        The namespace to use for the robot's topics, services, and actions
    robot_ip : str
        The IP address of the robot to connect to (required)
    rosbridge_port : int, default=9090
        The port number for the ROSBridge server on the robot
    topic_whitelist : list of str, default=[".*"]
        List of regex patterns for topics to expose from the robot
    service_whitelist : list of str, default=[".*"]
        List of regex patterns for services to expose from the robot
    action_whitelist : list of str, default=[".*"]
        List of regex patterns for actions to expose from the robot

    """

    def __init__(self):
        """
        Initialize the ROS2 bridge node.

        This constructor sets up a ROS2 node that serves as a bridge to communicate with a Niryo robot.
        It performs the following steps:
        1. Declares necessary parameters
        2. Retrieves configuration parameters (namespace, IP, port, whitelists)
        3. Creates a ROS2Driver instance to handle communication with the robot

        Parameters are automatically loaded from the node configuration.

        Raises:
            SystemExit: If the robot IP is not provided or if the driver creation fails
        """
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
        """
        Declare the node's parameters with their default values and descriptions.

        Parameters declared:
            - rosbridge_port (int): Port for the ROSBridge server (default: 9090)
            - robot_namespace (str): Robot's namespace (default: "")
            - robot_ip (str): Robot's IP address (default: "")
            - topic_whitelist (str[]): List of regex patterns for whitelisted topics (default: [".*"])
            - service_whitelist (str[]): List of regex patterns for whitelisted services (default: [".*"])
            - action_whitelist (str[]): List of regex patterns for whitelisted actions (default: [".*"])
        """
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

    def shutdown(self):
        """
        Shuts down the bridge by disconnecting the driver.

        This method is called when the node is shutting down. It checks if the driver
        attribute exists and, if so, calls its disconnect method to properly close
        any connections.
        """
        if hasattr(self, "_driver"):
            self._driver.disconnect()


def main():
    rclpy.init()

    node = Bridge()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Exception in bridge main loop: {e}")
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
