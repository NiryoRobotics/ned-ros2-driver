# /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .driver import ROS2Driver


class Bridge(Node):
    def __init__(self):
        super().__init__("ros2_bridge_node")

        self._init_parameters()

        namespaces = (
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )
        ips = self.get_parameter("robot_ips").get_parameter_value().string_array_value
        port = self.get_parameter("rosbridge_port").get_parameter_value().integer_value
        ros2_interfaces_package = (
            self.get_parameter("niryo_ros2_interfaces_package")
            .get_parameter_value()
            .string_value
        )

        # Check lengths
        if len(namespaces) != len(ips):
            self.get_logger().error(
                "robot_namespaces and robot_ips must have the same length"
            )
            return

        # Check for uniqueness
        if len(set(namespaces)) != len(namespaces):
            self.get_logger().error("robot_namespaces must be unique")
            return
        if len(set(ips)) != len(ips):
            self.get_logger().error("robot_ips must be unique")
            return

        self._drivers = {}

        robots = list(zip(namespaces, ips))
        for ns, ip in robots:
            self.get_logger().info(
                f"Creating driver for {ns} with IP: {ip} and port: {port}"
            )
            client = ROS2Driver(self, ns, ip, port, ros2_interfaces_package)
            self._drivers.update({ns: client})

    def __del__(self):
        for client in self._drivers.values():
            client.terminate()

    def _init_parameters(self):
        """
        Initialize parameters for the driver.
        """
        self.declare_parameter(
            "niryo_ros2_interfaces_package",
            "",
            descriptor=ParameterDescriptor(
                name="niryo_ros2_interfaces_package",
                type=ParameterType.PARAMETER_STRING,
                description="Name of the ROS2 package containing the Niryo interfaces",
            ),
        )
        self.declare_parameter(
            "rosbridge_port",
            9090,
            descriptor=ParameterDescriptor(
                name="rosbridge_port",
                type=ParameterType.PARAMETER_INTEGER,
                description="Port for the ROSBridge server",
            ),
        )
        self.declare_parameter(
            "robot_namespaces",
            [""],
            descriptor=ParameterDescriptor(
                name="robot_ips",
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of robot namespaces",
            ),
        )
        self.declare_parameter(
            "robot_ips",
            [""],
            descriptor=ParameterDescriptor(
                name="robot_ips",
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="List of robot IP addresses",
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
