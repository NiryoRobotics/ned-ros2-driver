string name # maximum length of dynamic_frame's name is 30 characters
string new_name
string description
bool belong_to_workspace

geometry_msgs/Point position
niryo_ned_ros2_interfaces/RPY rpy
geometry_msgs/Quaternion orientation

geometry_msgs/Point[] points
niryo_ned_ros2_interfaces/RobotState[] poses