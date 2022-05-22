# Setting up the robot to go
## Bridge
1. To launch the bridge navigate to the bridge_ws and make sure you have a `roscore` running in a different window.
2. Source your ROS1 and ROS2 installs with `. /opt/ros/ros_distro/setup.bash`
3. Set device to local host if necessary `export ROS_MASTER_URI=http://localhost:11311`
4. Run the bridge with `ros2 run ros1_bridge dynamic_bridge`

## ROS1 Nodes
1. Again, ensure you have a roscore running on a terminal.
2. Source the build with `. /devel/setup.bash` from ros1_ws
3. Run `roslaunch main robot.launch`

## ROS2 Nodes
1. Navigate to your ros2_ws
2. Source ROS2 and your build with `. /opt/ros/ros2_distro/setup.bash && . /install/setup.bash`
3. Run `ros2 launch robot_launch robot.launch.py`
4. (Optional) if you want to see the laser scan, run `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world laser_frame` and
`rviz2 ./install/rplidar_ros/share/rplidar_ros/rviz/rplidar.rviz` in another terminal

## Changing Parameters while Running
`ros2 param set /turtlesim background_r 150`
