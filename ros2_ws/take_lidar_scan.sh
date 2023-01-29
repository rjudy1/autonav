################################
# AutoNav 2023 Competition Robot
# File: take_lidar_scan.sh
# Purpose: launches the lidar laser_frame scan plotter node(s)
# Date Created:  23 Jan 2023
# Date Modified: 23 Jan 2023
################################
#colcon build
. install/setup.bash
clear
ros2 launch robot_launch lidar_plotter.launch.py