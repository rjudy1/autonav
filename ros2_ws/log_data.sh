################################
# AutoNav 2023 Competition Robot
# File: log_data.sh
# Purpose: launches the data logging node(s)
# Date Created:  1 Oct 2023
# Date Modified: 1 Oct 2023
################################
#colcon build
. install/setup.bash
clear
ros2 launch robot_launch data_logging.launch.py
