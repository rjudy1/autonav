from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from utils.utils import *


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_launch'),
        'config',
        'params.yml'
    )

    fsm_node = Node(
        package="master_fsm",
        executable="fsm",
        parameters=[
            config
        ]
    )

    ld = LaunchDescription()

    # HEADING
    # publishes gps
    gps_node = Node(
        package="heading",
        executable="gps_publisher",
        parameters=[
            config
        ]
    )
    # publishes turning
    teensy_node = Node(
        package="heading",
        executable="teensy",
        parameters=[
            config
        ]
    )
    # merges heading data
    fusion_node = Node(
        package="heading",
        executable="fusion",
        parameters=[
            config
        ]
    )

    # heading
    ld.add_action(gps_node)
    ld.add_action(teensy_node)
    ld.add_action(fusion_node)

    # master machine
    ld.add_action(fsm_node)

    return ld
