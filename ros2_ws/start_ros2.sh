. /opt/ros/galactic/setup.bash
colcon build
. install/setup.bash
clear
ros2 launch robot_launch robot.launch.py
