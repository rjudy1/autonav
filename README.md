# Cedarville University AutoNav Competition Robot 2022
This repository contains the code used for the Cedarville AutoNav 2022 robot. The robot is run through ROS2 nodes and takes a behavioral approach through a state machine.


## Quick Start


## Packages and Nodes
### Master FSM
The master fsm node resides in the master_fsm package. This controls the conditions for switching between states as well as sends commands to the motor controllers. 
It can be run with the command `ros2 run master_fsm fsm -p <parameter sets>`. Parameters available include:
- `/StartState` which can be selected from the STATE enum in the utils package.
- `/DefaultSpeed` which ranges from 0 to 64 but for stability has best results near 15.
- `/FollowingDirection` which indicates whether the robot should left or right line follow and is of value 0 or 1
- `/TimerRate` which sets how frequently the state change is checked in seconds.

### Utils
The utils package is not run directly but used by many of the nodes. It contains the state enums, various useful constants and functions, and custom messages.

### Launch
The robot_launch package contains the launch machine for the entire robot set of packages. Parameters can be set in the robot.launch.py file. To use defaults from the test machine, run `ros2 launch robot_launch robot.launch.py`

### GPS
The GPS node monitors the NMEA stream from the ZED F9P GPS and publishes the location data.

### Heading


### Wheels Controller


### Lines


### Obstacle Detection


### Realsense Camera


### RPLidar


