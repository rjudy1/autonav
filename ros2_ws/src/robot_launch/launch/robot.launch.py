from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    following_dir = 1  # right = 1, left = 0
    debug = True

    crop_top = 0
    crop_bottom = .2
    crop_side = .2

    # create launch description iwth initial camera launch file
    ld = LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        '/home/autonav/autonav/ros2_ws/src/vision/realsense-ros/realsense2_camera/launch/rs_launch.py'))])

    # launch lidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidarNode"
    )

    # launch line following node
    lines_node = Node(
        package="lines",
        executable="lines",
        parameters=[
            {'/LineDetectCropTop', crop_top},
            {'/LineDetectCropBottom', crop_bottom},
            {'/LineDetectCropSide', crop_side},
            {"/FollowingDirection", following_dir},
            {'/Debug', debug},
        ]
    )

    # launch rs2l_transform node
    transform_node = Node(
        package="rs2l_transform",
        executable="transform",
        parameters=[
            {'/LIDAR_Trim_Min', 1.57},  # radians
            {'/LIDAR_Trim_Max', 4.71},  # radians
            {'/PotholeDetectCropTop', crop_top},
            {'/PotholeDetectCropBottom', crop_bottom},
            {'/PotholeDetectCropSide', crop_side},
            {"/Debug", debug}
        ]
    )

    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(transform_node)

    return ld
