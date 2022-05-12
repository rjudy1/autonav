################################
# AutoNav 2022 Competition Robot
# Package: lines
# File: lines.py
# Purpose: controls line detection and line following
# Author: Modified from 2020-21 autonav team code for ROS2
# Date Modified: 11 May 2022
################################

# !/usr/bin/env python

import sys
sys.path.insert(1, '/home/autonav/autonav/')
sys.path.insert(1, '/home/autonav/autonav/ros2_ws/src/vision/lines/lines/')

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils import *


from line_detection import LineDetection
from line_following import LineFollowing
from std_msgs.msg import String

class Lines(Node):
    def __init__(self):
        super().__init__('lines')

        # States
        self.FOUND_LINE = "FOUND_LINE"
        self.LOST_LINE = "LOST_LINE"
        self.ALIGNED = "ALIGNED"
        self.NOT_ALIGNED = "NOT_ALIGNED"

        self.state = STATES.LINE_FOLLOWING
        self.LINE_CODE = "LIN,"

        # Subscribe to the camera color image
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)

        # Publish events that could change the robot state
        self.event_pub = self.create_publisher(String, "line_events", 10)

        # Publish distance from the center of the line
        self.motor_pub = self.create_publisher(String, "wheel_distance", 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(String, "state_topic", self.state_callback, 10)

        # Initialize Classes
        self.line_detection = LineDetection()
        self.line_following = LineFollowing()

        # Line Detection States Set
        self.line_detection_states = {STATES.OBJECT_AVOIDANCE_FROM_LINE, STATES.OBJECT_TO_LINE, STATES.FIND_LINE, STATES.LINE_ORIENT}

        # Set ROS Params

        self.get_logger().info("Waiting for image topics...")

    def image_callback(self, ros_image):
        image = bridge_image(ros_image, "bgr8")
        self.get_logger().info("CURRENT STATE: {}".format(self.state))

        # Line Followings
        if self.state == STATES.LINE_FOLLOWING:
            self.line_detection.reset()
            closeWindows(self.line_detection.window_handle)

            distance = self.line_following.image_callback(image)
            self.get_logger().warning("LINE_DISTANCE: " + str(distance))
            self.motor_pub.publish(String(self.LINE_CODE + str(distance)))

        # Line Detection
        elif self.state in self.line_detection_states:
            found_line, aligned = self.line_detection.image_callback(image, self.state)
            closeWindows(self.line_following.window_handle)
            self.get_logger().warning("Line Detection")
            if found_line:
                self.get_logger().warning(self.FOUND_LINE)
                self.event_pub.publish(String(self.FOUND_LINE))
            if aligned:
                self.get_logger().warning(self.ALIGNED)
                # self.line_following.reset()
                self.event_pub.publish(String(self.ALIGNED))
        else:
            closeWindows(self.line_following.window_handle)
            self.line_detection.reset()

    def state_callback(self, new_state):
        self.get_logger().info("New State Received ({}): {}".format(self.node_name, new_state.data))
        self.state = new_state.data


def main():
    rclpy.init()
    lines = Lines()

    try: rclpy.spin(lines)
    except KeyboardInterrupt:
        rclpy.logger.warn("Keyboard interrupt")


if __name__ == "__main__":
    main()
