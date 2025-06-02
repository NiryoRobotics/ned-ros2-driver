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

import pytest
import rclpy
from rclpy.node import Node
import launch_ros
import launch_pytest
from launch_testing.actions import ReadyToTest
from launch import LaunchDescription
import os
from threading import Event, Thread
import time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from launch.actions import TimerAction


ROBOT_NAMESPACE = "sim_robot"


@launch_pytest.fixture
def generate_test_description():
    test_whitelist = os.path.join(os.path.dirname(__file__), "test_whitelist.yaml")

    driver_node = launch_ros.actions.Node(
        package="niryo_ned_ros2_driver",
        executable="ros2_driver",
        name=f"ros2_driver_{ROBOT_NAMESPACE}",
        parameters=[
            {"robot_ip": "172.17.0.2"},
            {"robot_namespace": ROBOT_NAMESPACE},
            {"rosbridge_port": 9090},
            test_whitelist,
        ],
        output="screen",
    )

    timeout_action = TimerAction(
        period=5.0,
        actions=[
            ReadyToTest(),
        ],
    )

    return LaunchDescription(
        [
            driver_node,
            timeout_action,
        ]
    )


class DummyNode(Node):
    def __init__(self, name="integration_test_node"):
        super().__init__(name)
        self.spinning = True
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        self.get_logger().info(f"Created {name}")

    def _spin(self):
        """Spin the node in a background thread."""
        while self.spinning:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.01)  # Small sleep to avoid excessive CPU usage

    def shutdown(self):
        """Stop spinning and destroy the node."""
        self.get_logger().info("Shutting down TestNode")
        self.spinning = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)
        self.destroy_node()


@pytest.fixture
def spinning_node():
    if not rclpy.ok():
        rclpy.init()
    node = DummyNode()
    yield node
    node.shutdown()


@pytest.mark.launch(fixture=generate_test_description)
def test_topics_exist(spinning_node):
    expected_topics = [
        f"/{ROBOT_NAMESPACE}/joint_states",
        f"/{ROBOT_NAMESPACE}/tf",
        f"/{ROBOT_NAMESPACE}/tf_static",
        f"/{ROBOT_NAMESPACE}/niryo_robot/robot_state",
        f"/{ROBOT_NAMESPACE}/niryo_robot_status/robot_status",
        f"/{ROBOT_NAMESPACE}/niryo_robot_hardware_interface/hardware_status",
    ]

    found_all = False
    for _ in range(5):
        topic_list = spinning_node.get_topic_names_and_types()

        # Check if all expected topics are available
        found_topics = [topic for topic, _ in topic_list]
        missing_topics = [t for t in expected_topics if t not in found_topics]

        if not missing_topics:
            found_all = True
            break

        spinning_node.get_logger().info(
            f"Missing topics: {missing_topics}. Retrying..."
        )
        time.sleep(1)

    spinning_node.get_logger().info("Topics found:")
    for topic, types in topic_list:
        spinning_node.get_logger().info(f"  - {topic}: {types}")

    assert found_all, f"Not all expected topics were found. Missing: {missing_topics}"
    spinning_node.get_logger().info("All expected topics found!")


@pytest.mark.launch(fixture=generate_test_description)
def test_joint_states(spinning_node):
    """Test that joint states are being published."""
    joint_states_received = Event()

    # Subscribe to joint states
    def joint_states_callback(msg):
        joint_states_received.set()
        spinning_node.get_logger().info(
            f"Received joint states with {len(msg.position)} joints"
        )

    subscription = spinning_node.create_subscription(
        JointState, f"/{ROBOT_NAMESPACE}/joint_states", joint_states_callback, 10
    )

    joint_states_received.wait(timeout=5.0)

    spinning_node.destroy_subscription(subscription)

    assert (
        joint_states_received.is_set()
    ), "No joint states messages received within timeout"


@pytest.mark.launch(fixture=generate_test_description)
def test_tf_transforms(spinning_node):
    """Test that TF transforms are being published."""
    tf_messages_received = Event()

    # Subscribe to TF
    def tf_callback(msg):
        tf_messages_received.set()
        spinning_node.get_logger().info(
            f"Received TF message with {len(msg.transforms)} transforms"
        )

    subscription = spinning_node.create_subscription(
        TFMessage, f"/{ROBOT_NAMESPACE}/tf", tf_callback, 10
    )

    tf_messages_received.wait(timeout=5.0)

    spinning_node.destroy_subscription(subscription)

    assert tf_messages_received.is_set(), "No TF messages received within timeout"


@pytest.mark.launch(fixture=generate_test_description)
def test_tf_static(spinning_node):
    """Test that TF static transforms are being published."""
    tf_static_messages_received = Event()

    # Subscribe to TF static
    def tf_static_callback(msg):
        tf_static_messages_received.set()
        spinning_node.get_logger().info(
            f"Received TF static message with {len(msg.transforms)} transforms"
        )

    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_ALL,
    )

    subscription = spinning_node.create_subscription(
        TFMessage,
        f"/{ROBOT_NAMESPACE}/tf_static",
        tf_static_callback,
        qos_profile=qos,
    )

    tf_static_messages_received.wait(timeout=5.0)

    spinning_node.destroy_subscription(subscription)

    assert (
        tf_static_messages_received.is_set()
    ), "No TF static messages received within timeout"


@pytest.mark.launch(fixture=generate_test_description)
def test_service_availability(spinning_node):
    """Test that expected services are available."""
    expected_services = [
        f"/{ROBOT_NAMESPACE}/niryo_robot_tools_commander/set_tcp",
        f"/{ROBOT_NAMESPACE}/niryo_robot_tools_commander/enable_tcp",
        f"/{ROBOT_NAMESPACE}/niryo_robot_rpi/get_digital_io",
        f"/{ROBOT_NAMESPACE}/niryo_robot_rpi/set_digital_io",
        f"/{ROBOT_NAMESPACE}/niryo_robot_rpi/get_analog_io",
        f"/{ROBOT_NAMESPACE}/niryo_robot_rpi/set_analog_io",
    ]

    found_all = False
    for _ in range(5):
        service_list = spinning_node.get_service_names_and_types()

        # Check if any expected services are available
        found_services = [service for service, _ in service_list]
        missing_services = [s for s in expected_services if s not in found_services]

        if not missing_services:
            found_all = True
            break

        spinning_node.get_logger().info(
            f"Missing services: {missing_services}. Retrying..."
        )
        time.sleep(1)

    # Print all found services to help debug if test fails
    spinning_node.get_logger().info("Services found:")
    for service, types in service_list:
        spinning_node.get_logger().info(f"  - {service}: {types}")

    assert (
        found_all
    ), f"Not all expected services were found. Missing: {missing_services}"
    spinning_node.get_logger().info("All expected services found!")


@pytest.mark.launch(fixture=generate_test_description)
def test_multi_robot_namespace(spinning_node):
    """Test that the robot namespace is properly applied to topics."""
    topic_list = spinning_node.get_topic_names_and_types()
    spinning_node.get_logger().info(str(topic_list))
    topics = [t for t, _ in topic_list if t.startswith(f"/{ROBOT_NAMESPACE}/")]
    for topic in topics:
        spinning_node.get_logger().info(f"  - {topic}")
    assert len(topics) > 0, f"No topics found with /{ROBOT_NAMESPACE} namespace"
