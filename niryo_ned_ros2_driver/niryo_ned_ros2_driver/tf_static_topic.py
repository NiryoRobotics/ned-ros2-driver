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

import hashlib
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as Ros2Time
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from .topic import Topic


class StaticTFTopic(Topic):
    """
    StaticTFTopic provides ROS2 to ROS1 bridge functionality for static transform messages.
    This class extends the Topic base class to handle the publishing and conversion of static
    transform messages between ROS1 and ROS2 systems. It specifically configures appropriate
    Quality of Service (QoS) settings for static transform messages and implements caching
    to avoid republishing identical transforms.
    The class maintains a hash table of published transforms to efficiently detect and filter
    out unchanged transform messages, reducing network traffic and processing overhead.
        The implementation only supports one-way communication from ROS1 to ROS2.
        Transforms from ROS2 to ROS1 are not published (see _ros2_callback).
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._published_hashes = {}

    def _get_ros2_qos_for_topic(self, topic_name: str) -> QoSProfile:
        """
        Get the QoS profile for a given ROS2 topic.

        This method returns a QoSProfile configured for reliable delivery, transient local durability,
        and keeping all message history. This configuration is suitable for static transform
        messages that need to be available to late-joining subscribers.

        Args:
            topic_name : str
                The name of the ROS2 topic for which to configure the QoS profile.

        Returns:
            QoSProfile
                A QoS profile configured for reliable, transient local communication with complete history.
        """
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_ALL,
        )

    def _ros1_callback(self, msg_dict):
        """
        Process static transforms from ROS1 topic and publish them to ROS2.

        This callback receives transform messages from a ROS1 bridge, processes them,
        and publishes only those transforms that have changed since the last publication.

        Args:
            msg_dict : dict
                Dictionary representation of a ROS1 tf2_msgs/TFMessage containing transforms.
                Expected to have a "transforms" key with a list of transform dictionaries.

        Notes:
            - Transforms are hashed to detect changes and avoid republishing identical transforms.
            - Only transforms that are new or have changed will be published to ROS2.
        """
        if "transforms" not in msg_dict:
            return

        transforms_to_publish = []

        for t in msg_dict["transforms"]:
            parent = t["header"]["frame_id"].lstrip("/")
            child = t["child_frame_id"].lstrip("/")
            key = (parent, child)

            current_hash = self._hash_transform(t)

            if key in self._published_hashes:
                if self._published_hashes[key] == current_hash:
                    continue  # Unchanged, skip publishing

            self._published_hashes[key] = current_hash

            # Convert to ROS2 TransformStamped
            ros2_t = self._convert_to_ros2_transform(t, parent, child)
            transforms_to_publish.append(ros2_t)

        if transforms_to_publish:
            tf_msg = TFMessage(transforms=transforms_to_publish)
            self._ros2_publisher.publish(tf_msg)

    def _ros2_callback(self, msg):
        # Static TFs are not published from ROS2 to ROS1
        pass

    def _convert_to_ros2_transform(self, t, parent, child):
        """
        Converts a ROS1 transform dictionary to a ROS2 TransformStamped message.

        This method takes a transform dictionary (typically from ROS1 format), a parent frame ID,
        and a child frame ID, then converts it to a ROS2 TransformStamped message structure.

        Parameters:
            t (dict): The transform dictionary containing header, translation, and rotation information.
            parent (str): The parent frame ID to be set in the resulting TransformStamped message.
            child (str): The child frame ID to be set in the resulting TransformStamped message.

        Returns:
            TransformStamped: A ROS2 TransformStamped message populated with the transform data.

        Note:
            The method expects the transform dictionary to have the following structure:
            {
                "header": {"stamp": <time>},
                "transform": {
                    "translation": {"x": float, "y": float, "z": float},
                    "rotation": {"x": float, "y": float, "z": float, "w": float}
                }
            }
        """
        ros2_t = TransformStamped()
        ros2_t.header.stamp = self._convert_time(t["header"]["stamp"])
        ros2_t.header.frame_id = parent
        ros2_t.child_frame_id = child
        ros2_t.transform.translation.x = t["transform"]["translation"]["x"]
        ros2_t.transform.translation.y = t["transform"]["translation"]["y"]
        ros2_t.transform.translation.z = t["transform"]["translation"]["z"]
        ros2_t.transform.rotation.x = t["transform"]["rotation"]["x"]
        ros2_t.transform.rotation.y = t["transform"]["rotation"]["y"]
        ros2_t.transform.rotation.z = t["transform"]["rotation"]["z"]
        ros2_t.transform.rotation.w = t["transform"]["rotation"]["w"]
        return ros2_t

    def _convert_time(self, stamp_dict):
        """
        Convert a timestamp dictionary to a ROS2 Time object.

        Args:
            stamp_dict (dict): Dictionary containing the timestamp information with keys:
                - secs (int): Seconds component of the timestamp, defaults to 0 if not present
                - nsecs (int): Nanoseconds component of the timestamp, defaults to 0 if not present

        Returns:
            Ros2Time: A ROS2 Time object
        """
        secs = stamp_dict.get("secs", 0)
        nsecs = stamp_dict.get("nsecs", 0)
        return Ros2Time(sec=secs, nanosec=nsecs)

    def _hash_transform(self, t):
        """
        Generate a hash for a transform message.

        This method creates a unique hash string based on the transform message's contents,
        including the frame IDs and transformation values.

        Args:
            t : dict
                A transform message dictionary with the following structure:
                {
                    'header': {'frame_id': str},
                    'child_frame_id': str,
                    'transform': {
                        'translation': {'x': float, 'y': float, 'z': float},
                        'rotation': {'x': float, 'y': float, 'z': float, 'w': float}
                    }
                }

        Returns:
            str: MD5 hash of the transform's string representation
        """
        s = (
            f"{t['header']['frame_id'].lstrip('/')}_{t['child_frame_id'].lstrip('/')}_"
            f"{t['transform']['translation']['x']}_{t['transform']['translation']['y']}_{t['transform']['translation']['z']}_"
            f"{t['transform']['rotation']['x']}_{t['transform']['rotation']['y']}_{t['transform']['rotation']['z']}_{t['transform']['rotation']['w']}"
        )
        return hashlib.md5(s.encode()).hexdigest()
