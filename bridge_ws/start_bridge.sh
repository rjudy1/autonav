. /opt/ros/noetic/setup.bash
. /opt/ros/galactic/setup.bash
#colcon build
. install/setup.bash
roscore&
ros2 run ros1_bridge dynamic_bridge
