################################
# AutoNav 2023 Competition Robot
# File: lidar_plotter.py
# Purpose: creates and launches the lidar laser_frame scan plotter node
# Date Created:  23 Jan 2023
# Date Modified: 23 Jan 2023
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

    lidar_plotter_node = Node(
        package = "lidar_plotter",
        executable="plot_scan",
        parameters=[]
    )

    ld.add_action(lidar_plotter_node)

    return ld