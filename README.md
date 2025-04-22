# ROS2 driver

Faire très attention à update les messages ros2 du driver au besoin !

Add venv to pythonpath
PYTHONPATH=/home/thomas/development/git/niryo/ros2_driver_ws/venv/lib/python3.12/site-packages

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y



ros2 run pour 1 driver avec params

ros2 run niryo_ned_ros2_driver ros2_driver --ros-args -p robot_ip:=192.168.1.137 -p topic_whitelist:="[\"/joint_states\", \"/niryo_robot_follow_joint_trajectory_controller/.*\"]"
regex

ros2 launch pour plusieurs drivers avec params

ros2 launch niryo_ned_ros2_driver driver.launch.py whitelist_params_file:=src/ros2-driver/niryo_ned_ros2_driver/config/whitelist.yaml