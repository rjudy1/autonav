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

    fsm_node = Node(
        package="master_fsm",
        executable="fsm",
        parameters=[
            {'/DefaultSpeed': 10},
            {'/FollowingDirection': following_dir},
            {'/TimerRate': .05},
            {'/StartState': STATE.LINE_FOLLOWING},
        ]
    )

    # VISION
    # create launch description with initial camera launch file
    # disgusting but not quite sure how to make local path
    ld = LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'))
        )]
    )
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
        package="path_detection",
        executable="lines",
        parameters=[
            {'/LineDetectCropTop': crop_top},  # fraction to remove
            {'/LineDetectCropBottom': crop_bottom},
            {'/LineDetectCropSide': crop_side},
            {"/FollowingDirection": following_dir},
            {'/UseYellow': False},
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
            {'/ObstacleDetectDistance': 3.0},  # meters
            {'/FollowingDirection': following_dir},
            {"/Debug": True},
        ]
    )

    # HEADING
    # publishes gps
    gps_node = Node(
        package="heading",
        executable="gps_publisher",
        parameters=[
            {'/WaypointLat1': 0.0},
            {'/WaypointLon1': 0.0},
            {'/WaypointLat2': 0.0},
            {'/WaypointLon2': 0.0},
            {'/WaypointLat3': 0.0},
            {'/WaypointLon3': 0.0},
            {'/WaypointLat4': 0.0},
            {'/WaypointLon4': 0.0},
            {'/ExitAngle': 0.2},  # radians
            {'/GPSFollowGoal': 1.0},
            {'/LineToGPSTrans': 5.0},
            {'/Port': '/dev/ttyACM0'},
        ]
    )
    # publishes turning
    encoder_node = Node(
        package="heading",
        executable="encoder_pub",
        parameters=[
            {'/TeensyEncodersPort': '/dev/ttyACM1'},
            {'/TeensyBaudrate': 115200},
            {'/TeensyUpdateDelay': .01},
            {'/Debug': False},
        ]
    )
    # merges heading data
    fusion_node = Node(
        package="heading",
        executable="fusion",
        parameters=[
            {'/InitialHeading': 0.0},
            {'/EncoderWeight': 1.0},
            {'/Debug': True},
        ]
    )

    # MOTOR CONTROL
    motor_node = Node(
        package="wheels_controller",
        executable="controller",
        parameters=[
            {'/FollowingDirection': following_dir},
            {'/LineDist': 0.25},
            {'/SideObjectDist': 0.5},
            {'/DefaultSpeed': 25.0},
            {'/BoostIncrease': 2},
            {'/BoostCountThreshold': 20},
            {'/LineBoostMargin': 30.0},
            {'/GPSBoostMargin': 0.1745},
            {'/Port': '/dev/ttyUSB0'},
        ]
    )

    # vision
    # ld.add_action(lidar_node)
    ld.add_action(lines_node)
    # ld.add_action(obstacles_node)

    # heading
    # ld.add_action(gps_node)
    # ld.add_action(encoder_node)
    # ld.add_action(fusion_node)

    # motors
    ld.add_action(motor_node)

    # master machine
    ld.add_action(fsm_node)

    return ld
