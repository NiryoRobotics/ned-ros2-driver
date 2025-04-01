import hashlib
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as Ros2Time

from .topic import Topic


class StaticTFTopic(Topic):
    """
    Class to handle static TF topics.
    It subscribes to a ROS1 topic and publishes the transforms to a ROS2 topic.
    It also handles deduplication of transforms to avoid sending the same transform multiple times.
    A hash on the transform data is used to identify unique transforms.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Deduplication dictionary: (frame_id, child_frame_id)-> hash
        self._published_hashes = {}

    def _ros1_callback(self, msg_dict):
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

    def _convert_to_ros2_transform(self, t, parent, child):
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
        secs = stamp_dict.get("secs", 0)
        nsecs = stamp_dict.get("nsecs", 0)
        return Ros2Time(sec=secs, nanosec=nsecs)

    def _hash_transform(self, t):
        s = (
            f"{t['header']['frame_id'].lstrip('/')}_{t['child_frame_id'].lstrip('/')}_"
            f"{t['transform']['translation']['x']}_{t['transform']['translation']['y']}_{t['transform']['translation']['z']}_"
            f"{t['transform']['rotation']['x']}_{t['transform']['rotation']['y']}_{t['transform']['rotation']['z']}_{t['transform']['rotation']['w']}"
        )
        return hashlib.md5(s.encode()).hexdigest()
