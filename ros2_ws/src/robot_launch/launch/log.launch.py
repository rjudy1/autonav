################################
# AutoNav 2023 Competition Robot
# File: log.launch.py
# Purpose: launches the data logging node(s)
# Date Created:  1 Oct 2023
# Date Modified: 1 Oct 2023
################################

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os
from utils.utils import *

def generate_launch_description():

    ld = LaunchDescription()

    logging_node = Node(
        package = "logging",
        executable="logger",
        parameters=[]
    )

    ld.add_action(logging_node)

    return ld