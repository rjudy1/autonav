# Cedarville University AutoNav Competition Robot 2022
This repository contains the code used for the Cedarville AutoNav 2022 robot. The robot is run through ROS2 nodes and takes a behavioral approach through a state machine.

## Quick Start
1. Navigate to `cd ros2_ws` and build with `colcon build`
2. Source ROS2 and your build with `. /opt/ros/ros2_distro/setup.bash && . /install/setup.bash`
3. Run `ros2 launch robot_launch robot.launch.py`. If this doesn't run or throws errors, view the robot.launch.py file to modify the port parameters.
4. (Optional) if you want to see the laser scan, run the following commands in new terminals:
- `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world laser_frame`
- `rviz2 ./install/rplidar_ros/share/rplidar_ros/rviz/rplidar.rviz`

## Packages and Nodes
### Master FSM
The master fsm node resides in the master_fsm package. This controls the conditions for switching between states as well as sends commands to the motor controllers. 
It can be run individually from the launch file with the command `ros2 run master_fsm fsm -p <parameter sets>`. Parameters available include:
- `/StartState` which can be selected from the STATE enum in the utils package.
- `/DefaultSpeed` which ranges from 0 to 64 but for stability has best results near 15.
- `/FollowingDirection` which indicates whether the robot should left or right line follow and is of value 0 or 1
- `/TimerRate` which sets how frequently the state change is checked in seconds.

### Utils
The utils package is not run directly but used by many of the nodes. It contains the state enums, various useful constants and functions, and custom messages.

### Custom Messages
The custom_msgs package is also not run directly but is used by the nodes. It messages are primarily for the encoders and GPS fusion.

### Launch
The robot_launch package contains the launch machine for the entire robot set of packages. Parameters can be set in the robot.launch.py file. To use defaults from the test machine, run `ros2 launch robot_launch robot.launch.py`

### GPS
The GPS node `gps_publisher` belongs to the `heading` package which monitors the NMEA stream from the ZED F9P GPS and publishes the location data to `gps_events` and `gps_heading` where the fusion node and the master state machine use these to maintain heading. 
- `/WaypointLat1` which sets the latitude of the first waypoint. These go up to four points.
- `/WaypointLon1` which sets the longitude of the waypoints, changing number for points
- `/ExitAngle`
- `/GPSFollowGoal` for how close to get to the point before considering it an arrival
- `/LineToGPSTrans`
- `/Port` which is the port the GPS is on 

### Encoders
The encoders publish to the `encoder_data` topic. They can track the distance the wheels travel to the centimeter and report the change since last message.
- `/TeensyEncodersPort`
- `/TeensyBaudrate`
- `TeensyUpdateDelay`
- `/Debug`

### Fusion for Heading
The fusion node publishes to the wheels and can publish its own `gps_event` messages. It basically weights the encoder and GPS provided headings to maintain on course.
- `/InitialHeading'` sets the initial angle relative to lat/long
- `/EncoderWeight` which weights the encoder versus the gps heading
- `/Debug`

### Wheels Controller
The wheels controller receives the wheel_distance commands and obstacles events and changes the wheel speeds accordingly.
- `FollowingDirection`
- `/LineDist`
- `/SideObjectDist`
- `/DefaultSpeed': 25`
- `/BoostIncrease'`
- `/BoostCountThreshold`
- `/LineBoostMargin`
- `/GPSBoostMargin`
- `/Port`

### Lines
The lines node belongs to the path_detection package. It detects and maintains line following. The line parameters modify the camera cropping, color to follow, and following direction.
- `/LineDetectionCropTop` which cuts off a fraction of the top of the image
- `/LineDetectionCropBottom`
- `/LineDetectionCropSide` which crops both sides
- `/FollowingDirection` which indicates left and right wall following
- `/UseYellow` which allows yellow line following instead of white for outdoor parking lot tests
- `/Debug` which enables some of the debug output

### Obstacle Detection
The obstacles node belongs to the path_detection package. It is responsible for mapping vision camera potholes onto the LIDAR scan and publishing this obstacle data. Its parameters are similar to the lines node with some additional obstacle limits.
- `/LIDARTrimMin` which sets the left edge on the unit circle of where to cut off the scan for the robot body
- `/LIDARTrimMax` which sets the max angle of which to cut off
- `/ObstacleFOV` which sets the area in front of the robot to consider a dangerous obstacle
- `/PotholeDetectCropTop`
- `/PotholeDetectCropBottom`
- `/PotholeDetectCropSide`
- `/PotholeBufferSize` which maintains the history size used for the pothole conclusive detection
- `/ObstacleDetectDistance` which says how many meters at which to detect obstacles
- `/Debug` to display debug data

### Realsense Camera
The RealSense camera is run through a provided package. It reports on the `/camera/color/image_raw` topic which is received by the vision nodes. If desiring to run it outside of the master launch file, run `ros2 launch realsense2_camera rs_launch.py`

### RPLidar
The RPLIDAR also is controlled through a provided package. It spins and publishes the LIDAR scans. You may have to set the port with the parameters:
- `serial_port` which is by default `/dev/ttyUSB0`


## Helpful Tools/Tips
- If wanting to use ROS1 versions of packages or rosbags see the helpful ros1 to ros2 bridge at [https://github.com/ros2/ros1_bridge](RosBridge)
- To change parameters while running see `ros2 param set <node> <parameter> <parameter_value>
