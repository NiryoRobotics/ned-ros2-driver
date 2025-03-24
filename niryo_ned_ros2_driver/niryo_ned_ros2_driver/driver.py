# /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import roslibpy


class ROS2Driver(Node):
    def __init__(self):
        super().__init__("ros2_driver_node")

        self.init_all_parameters()

        namespaces = (
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )
        ips = self.get_parameter("robot_ips").get_parameter_value().string_array_value
        port = self.get_parameter("rosbridge_port").get_parameter_value().integer_value

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

        self._rosbridge_clients = {}

        robots = list(zip(namespaces, ips))
        for ns, ip in robots:
            self.get_logger().info(
                f"Creating client for {ns} with IP: {ip} and port: {port}"
            )
            client = roslibpy.Ros(host=ip, port=port)
            client.run()
            self._rosbridge_clients.update({ns: client})

    def __del__(self):
        for client in self._rosbridge_clients.values():
            client.terminate()

    def init_all_parameters(self):
        """
        Initialize all parameters for the driver.
        """
        self.declare_parameter("rosbridge_port", 9090)
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

    node = ROS2Driver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
