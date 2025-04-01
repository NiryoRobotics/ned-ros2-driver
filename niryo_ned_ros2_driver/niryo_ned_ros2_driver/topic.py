# /usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.set_message import set_message_fields

import roslibpy

from .models import ROSTypes
from .utils import normalize_ros1_type_to_ros2

LATCHED_ROS1_TOPICS = {
    "/niryo_robot_status/robot_status",
    "/niryo_robot_led_ring/led_ring_status",
    "/niryo_robot/max_velocity_scaling_factor",
    "/niryo_robot/max_acceleration_scaling_factor",
    "/niryo_robot_arm_commander/trajectory_list",
    "/niryo_robot_poses_handlers/dynamic_frame_list",
    "/niryo_robot_poses_handlers/workspace_list",
    "/niryo_robot_poses_handlers/pose_list",
    "/visualization_marker_array",
    "/niryo_robot_programs_manager_v2/program_list",
    "/niryo_robot_rpi/digital_io_state",
    "/niryo_robot_rpi/analog_io_state",
    "/niryo_robot_rpi/pause_state",
    "/niryo_robot/rpi/is_button_pressed",
    "/niryo_robot/rpi/led_state",
    "/niryo_robot/rpi/is_button_pressed",
    "/niryo_robot_sound/sound_database",
    "/niryo_robot_sound/sound",
    "/niryo_robot_sound/volume",
    "/niryo_robot_tools_commander/current_id",
    "/niryo_robot_tools_commander/tcp",
    "/niryo_robot_vision/visualization_marker",
    "/niryo_robot_vision/camera_intrinsics",
    "/niryo_robot_vision/video_stream_parameters",
}


class Topic:
    def __init__(
        self,
        node: Node,
        topic_name: str,
        topic_types: ROSTypes,
        prefix: str,
        rosbridge_client: roslibpy.Ros,
    ):
        self._node = node
        self._topic_name = topic_name
        self._topic_types = topic_types
        self._prefix = prefix
        self._qos = self._get_ros2_qos_for_topic(topic_name)
        self._rosbridge_client = rosbridge_client

        self._node.get_logger().debug(
            f"Creating topic bridge for {topic_name} ({topic_types.ros1_type} → {topic_types.ros2_type})"
        )

        self._ros2_type_str = topic_types.ros2_type
        self._ros2_msg_class = get_message(self._ros2_type_str)

        self._ros2_subscriber = self._create_ros2_subscriber()
        self._ros2_publisher = self._create_ros2_publisher()
        self._ros1_subscriber = self._create_ros1_subscriber()
        self._ros1_publisher = self._create_ros1_publisher()

    def _create_ros2_publisher(self):
        """
        Create a ROS2 publisher for the topic.
        """
        return self._node.create_publisher(
            self._ros2_msg_class,
            f"{self._prefix}{self._topic_name}",
            self._qos,
        )

    def _create_ros2_subscriber(self):
        """
        Create a ROS2 subscriber for the topic.
        """
        return self._node.create_subscription(
            self._ros2_msg_class,
            f"{self._prefix}{self._topic_name}",
            self._ros2_callback,
            self._qos,
        )

    def _create_ros1_publisher(self):
        """
        Create a ROS1 publisher for the topic.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        )

    def _create_ros1_subscriber(self):
        """
        Create a ROS1 subscriber for the topic.
        """
        return roslibpy.Topic(
            self._rosbridge_client, self._topic_name, self._topic_types.ros1_type
        ).subscribe(self._ros1_callback)

    def _ros1_callback(self, msg_dict):
        """
        Callback for the ROS1 subscriber.
        Converts the dictionary from roslibpy into a ROS2 message.
        """
        ros2_msg = self._ros2_msg_class()
        normalize_ros1_type_to_ros2(msg_dict, self._ros2_type_str)
        try:
            set_message_fields(ros2_msg, msg_dict)
            self._ros2_publisher.publish(ros2_msg)
        except Exception as e:
            self._node.get_logger().error(
                f"Failed to convert ROS1 → ROS2 message for topic '{self._topic_name}': {e}"
            )

    def _ros2_callback(self, ros2_msg):
        """
        Callback for the ROS2 subscriber.
        """

        # self._ros1_publisher.publish(msg)

    def _get_ros2_qos_for_topic(self, topic_name: str) -> QoSProfile:
        """
        Returns the appropriate ROS2 QoS profile for a given topic name.
        """
        if topic_name in LATCHED_ROS1_TOPICS:
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
        else:
            # Default to non-latched behavior
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
