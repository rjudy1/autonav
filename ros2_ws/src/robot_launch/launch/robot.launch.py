from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import math
import os
import sys
sys.path.insert(1, '/home/autonav/autonav/')
from utils import DIRECTION

def generate_launch_description():
    print(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    following_dir = DIRECTION.RIGHT
    crop_top = 0.0
    crop_bottom = .2
    crop_side = .2

    # create launch description with initial camera launch file
    # disgusting but not quite sure how to make local path
    ld = LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        '/home/autonav/autonav/ros2_ws/src/vision/realsense-ros/realsense2_camera/launch/rs_launch.py'))])

    # launch lidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidarNode",
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
        ]
    )

    # launch line following node
    lines_node = Node(
        package="lines",
        executable="lines",
        parameters=[
            {'/LineDetectCropTop': crop_top},
            {'/LineDetectCropBottom': crop_bottom},
            {'/LineDetectCropSide': crop_side},
            {"/FollowingDirection": following_dir},
            {'/Debug': True},
        ]
    )

    # launch rs2l_transform node
    transform_node = Node(
        package="rs2l_transform",
        executable="transform",
        parameters=[
            {'/LIDARTrimMin': 1.57},  # radians
            {'/LIDARTrimMax': 4.71},  # radians
            {'/ObstacleFOV': math.pi / 6.0},  # radians
            {'/PotholeDetectCropTop': crop_top},
            {'/PotholeDetectCropBottom': crop_bottom},
            {'/PotholeDetectCropSide': crop_side},
            {'/PotholeBufferSize': 5},
            {"/Debug": True},
        ]
    )

    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(transform_node)

    # set debug
    if True:
        sub_node = Node(
            package="subscriber",
            executable="sub"
        )
        ld.add_action(sub_node)

    return ld
