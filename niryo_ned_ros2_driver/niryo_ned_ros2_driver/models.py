from dataclasses import dataclass


@dataclass
class ROSTypes:
    ros1_type: str
    ros2_type: str

    def __repr__(self):
        return f"ROS1 Type: {self.ros1_type}, ROS2 Type: {self.ros2_type}"
