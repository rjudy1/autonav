from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import math
import os
from utils.utils import *


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_launch'),
        'config',
        'params.yml'
    )

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
            # config
        ]
    )

    # # VISION
    # # create launch description with initial camera launch file
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
            {'serial_port': '/dev/LIDAR_PORT'},
            # config
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
            # config
        ]
    )
    # launch obstacle detection
    obstacles_node = Node(
        package="path_detection",
        executable="obstacles",
        parameters=[
            {'/LIDARTrimMin': 1.31},  # radians
            {'/LIDARTrimMax': 4.97},  # radians
            {'/ObstacleFOV': math.pi / 6.0},  # radians
            {'/PotholeDetectCropTop': crop_top},
            {'/PotholeDetectCropBottom': crop_bottom},
            {'/PotholeDetectCropSide': crop_side},
            {'/PotholeBufferSize': 5},
            {'/ObstacleDetectDistance': 2.0},  # meters
            {'/FollowingDirection': following_dir},
            {"/Debug": False},
            # config
        ]
    )

    # HEADING
    # publishes gps
    gps_node = Node(
        package="heading",
        executable="gps_publisher",
        parameters=[
            {'/WaypointLat1': 39.4481695},
            {'/WaypointLon1': -83.4882655},
            {'/WaypointLat2': 0.0},
            {'/WaypointLon2': 0.0},
            {'/WaypointLat3': 0.0},
            {'/WaypointLon3': 0.0},
            {'/WaypointLat4': 0.0},
            {'/WaypointLon4': 0.0},
            {'/ExitAngle': 0.2},  # radians
            {'/GPSFollowGoal': 3.0},
            {'/LineToGPSTrans': 5.0},
            {'/Port': '/dev/GPS_PORT'},
            # config
        ]
    )
    # publishes turning
    encoder_node = Node(
        package="heading",
        executable="encoder_pub",
        parameters=[
            {'/TeensyEncodersPort': '/dev/TEENSY_PORT'},
            {'/TeensyBaudrate': 115200},
            {'/TeensyUpdateDelay': .05},
            {'/Debug': True},
            # config
        ]
    )
    # merges heading data
    fusion_node = Node(
        package="heading",
        executable="fusion",
        parameters=[
            {'/InitialHeading': -1.5},
            {'/EncoderWeight': .90},
            {'/Debug': True},
            # config
        ]
    )

    # MOTOR CONTROL
    motor_node = Node(
        package="wheels_controller",
        executable="controller",
        parameters=[
            {'/FollowingDirection': following_dir},
            {'/LineDist': 0.3},
            {'/SideObjectDist': 0.5},
            {'/DefaultSpeed': 28.0},
            {'/BoostIncrease': 2},
            {'/BoostCountThreshold': 20},
            {'/LineBoostMargin': 30.0},
            {'/GPSBoostMargin': 0.1745},
            {'/Port': '/dev/MOTOR_PORT'},
            {'/Debug': True},
            # config
        ]
    )

    # vision
    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(obstacles_node)

    # heading
    ld.add_action(gps_node)
    ld.add_action(encoder_node)
    ld.add_action(fusion_node)

    # motors
    ld.add_action(motor_node)

    # master machine
    ld.add_action(fsm_node)

    return ld
