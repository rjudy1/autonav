# LAUNCHES VISION ASPECT OF ROBOT

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import math
import os
from utils.utils import *


def generate_launch_description():
    following_dir = DIRECTION.LEFT
    crop_top = 0.0
    crop_bottom = .2
    crop_side = .2

    # VISION
    # create launch description with initial camera launch file
    # disgusting but not quite sure how to make local path
    ld = LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'))
        )]
    )
    # # launch lidar node
    # lidar_node = Node(
    #     package="rplidar_ros",
    #     executable="rplidarNode",
    #     parameters=[
    #         {'serial_port': '/dev/ttyUSB1'},
    #     ]
    # )
    # launch line following node
    lines_node = Node(
        package="path_detection",
        executable="lines",
        parameters=[
            {'/LineDetectCropTop': crop_top},  # fraction to remove
            {'/LineDetectCropBottom': crop_bottom},
            {'/LineDetectCropSide': crop_side},
            {"/FollowingDirection": following_dir},
            {'/UseYellow': True},
            {'/Debug': True},
        ]
    )
    # launch obstacle detection
    obstacles_node = Node(
        package="path_detection",
        executable="obstacles",
        parameters=[
            {'/LIDARTrimMin': 1.57},  # radians
            {'/LIDARTrimMax': 4.71},  # radians
            {'/ObstacleFOV': math.pi / 6.0},  # radians
            {'/PotholeDetectCropTop': crop_top},
            {'/PotholeDetectCropBottom': crop_bottom},
            {'/PotholeDetectCropSide': crop_side},
            {'/PotholeBufferSize': 5},
            {'/ObstacleDetectDistance': 2.0},  # meters
            {'/FollowingDirection': following_dir},
            {"/Debug": True},
        ]
    )

    # vision
    # ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(obstacles_node)

    return ld
