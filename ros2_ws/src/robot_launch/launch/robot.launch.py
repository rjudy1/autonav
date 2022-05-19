from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        '/home/autonav/autonav/ros2_ws/src/vision/realsense-ros/realsense2_camera/launch/rs_launch.py'))])

    # launch camera node with own launch file
    # included_camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     '/home/autonav/autonav/ros2_ws/src/vision/realsense-ros/realsense2_camera/launch/rs_launch.py'))
        # package='realsense2_camera',
        # launch='rs_launch.py',
        # arguments=[...]


    # launch lidar node
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidarNode"
    )

    # launch line following node
    lines_node = Node(
        package="lines",
        executable="lines"
    )

    # launch rs2l_transform node
    transform_node = Node(
        package="rs2l_transform",
        executable="transform",
    )

    ld.add_action(lidar_node)
    ld.add_action(lines_node)
    ld.add_action(transform_node)

    return ld