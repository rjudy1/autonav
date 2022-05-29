from launch import LaunchDescription
from launch_ros.actions import Node
from utils.utils import *


def generate_launch_description():
    following_dir = DIRECTION.LEFT

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

    ld = LaunchDescription()

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
        ]
    )
    # publishes turning
    encoder_node = Node(
        package="heading",
        executable="encoder_pub",
        parameters=[
            {'/TeensyEncodersPort': '/dev/TEENSY_PORT'},
            {'/TeensyBaudrate': 115200},
            {'/TeensyUpdateDelay': .02},
            {'/Debug': False},
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
            {'/DefaultSpeed': 25.0},
            {'/BoostIncrease': 2},
            {'/BoostCountThreshold': 20},
            {'/LineBoostMargin': 30.0},
            {'/GPSBoostMargin': 0.1745},
            {'/Port': '/dev/MOTOR_PORT'},
            {'/Debug': True},
        ]
    )

    # heading
    ld.add_action(gps_node)
    ld.add_action(encoder_node)
    ld.add_action(fusion_node)

    # motors
    ld.add_action(motor_node)

    # master machine
    ld.add_action(fsm_node)

    return ld
