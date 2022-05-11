################################
# AutoNav 2022 Competition Robot
# Package: rs2l_transform
# File: transform.py
# Purpose: takes lidar and real sense camera messages and maps real sense onto lidar then republishes to new topic
# Author: Rachael Judy
# Date Modified: 11 May 2022
################################

# !/usr/bin/env python

import cv2
import math
import numpy as np
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# useful functions pulled from autonav_class.py
def hsv_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 65, 100])
    upper = np.array([179, 255, 255])
    return cv2.inRange(hsv, lower, upper)


def cvDisplay(image, handle, handleArr):
    cv2.imshow(handle, image)
    cv2.waitKey(2)
    if handle not in handleArr:
        handleArr.append(handle)


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'mlidar', 10)
        #        timer_period = 0.5  # seconds
        #        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.bridge = CvBridge()

        self.declare_parameter('/LineDetectBufferSize', 10)
        self.declare_parameter('/LineDetectBufferFill', 0.8)
        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)
        self.declare_parameter('/LineDetectMaxWhite', 0.5)
        self.declare_parameter('/LineDetectMinSlope', 0.9)
        self.declare_parameter('/LineDetectMinLineLength', 0.35)
        self.declare_parameter('/LineDetectLineDistance', 150)
        self.declare_parameter('/Debug', True)

        # Read ROS Params - Line Detection
        self.BUFF_SIZE = self.get_parameter('/LineDetectBufferSize').value
        self.BUFF_FILL = self.get_parameter('/LineDetectBufferFill').value
        self.CROP_TOP = self.get_parameter('/LineDetectCropTop').value
        self.CROP_BOTTOM = self.get_parameter('/LineDetectCropBottom').value
        self.CROP_SIDE = self.get_parameter('/LineDetectCropSide').value
        self.MAX_WHITE = self.get_parameter('/LineDetectMaxWhite').value
        self.MIN_SLOPE = self.get_parameter('/LineDetectMinSlope').value
        self.MIN_LINE_LENGTH = self.get_parameter('/LineDetectMinLineLength').value
        self.LINE_DISTANCE = self.get_parameter('/LineDetectLineDistance').value
        self.line_history = [0] * self.BUFF_SIZE
        self.DEBUG_MODE = self.get_parameter('/Debug').value

        # Subscribe to the camera color image
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)

        # line detection parameters
        self.history_idx = 0
        self.slope = None
        self.aligned = False
        self.found_line = False
        self.window_handle = []


    # Use cv_bridge() to convert the ROS image to OpenCV format
    def bridge_image(self, ros_image, format):
        try: cv_image = self.bridge.imgmsg_to_cv2(ros_image, format)
        except CvBridgeError as e: rospy.logerr("CvBridge could not convert images from ROS to OpenCV")
        return cv_image

    def bridge_image_pub(self, cv_image, format):
        try: ros_image = self.bridge.cv2_to_imgmsg(cv_image, format)
        except CvBridgeError as e: rospy.logerr("CvBridge could not convert images from OpenCV to ROS")
        return ros_image


    def image_callback(self, image):
        # Save Dimensions

        y, x = image.height, image.width
        image = self.bridge_image(image, "bgr8")



        # Slice Edges
        image = image[int(y*self.CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]
        cvDisplay(image, 'Line Detection Opened Image Color', self.window_handle)


        # Remove Shadows
        image = hsv_filter(image)

        # Discard Oversaturated Images
        if np.count_nonzero(image) < image.shape[0]*image.shape[1]*self.MAX_WHITE:
            # Open Image
            morph = cv2.morphologyEx(image, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
            line, coords = self.determine_line(morph)
            if self.DEBUG_MODE:
                if line:
                    x1, y1, x2, y2 = coords
                    morph = cv2.cvtColor(morph, cv2.COLOR_GRAY2RGB)
                    cv2.line(morph, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)

                cvDisplay(morph, 'Line Detection Opened Image', self.window_handle)
        else:
            self.get_logger().warning('Discarded image')
            self.update_history(0)

        found_line = self.determine_state()
        aligned = self.determine_orientation()

        if self.DEBUG_MODE:
            self.get_logger().info(f'Found line: {found_line}, aligned: {aligned}')
        return found_line, aligned

    def determine_line(self, image, state=''):
        # if state == "OBJECT_TO_LINE":
        #     lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH/2), maxLineGap=10)
        # else:
        lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH), maxLineGap=10)

        if lines is not None:
            self.update_history(1)
            x1, y1, x2, y2 = lines[0][0]
            self.slope = self.get_slope(x1, y1, x2, y2)
            self.distance = self.get_distance(image.shape[1], image.shape[0], [x1, y1, x2, y2])
            self.get_logger().info("LINE SLOPE: {} | LINE DISTANCE: {}".format(self.slope, self.distance))
            return True, [x1, y1, x2, y2]
        else: self.update_history(0)
        return False, [0, 0, 0, 0]

    def get_slope(self, x1, y1, x2, y2):
        return float(abs(y2-y1)) / float(abs(x2-x1)+0.00001)

    def get_distance(self, y, x, coords):
        x1, y1, x2, y2 = coords
        line_center = [(x1 + x2) // 2, (y1 + y2) // 2]
        robot_center = [x // 2, y]
        dist_left = math.sqrt(pow(x1 - robot_center[0], 2) + pow(y1 - robot_center[1], 2))
        dist_right = math.sqrt(pow(x2 - robot_center[0], 2) + pow(y2 - robot_center[1], 2))
        dist_center = dist_left = math.sqrt(pow(line_center[0] - robot_center[0], 2) + pow(line_center[1] - robot_center[1], 2))
        return min([dist_left, dist_right, dist_center])

    def determine_state(self):
        if not self.found_line and (self.line_history.count(1) >= self.BUFF_FILL * self.BUFF_SIZE) and (self.distance <= self.LINE_DISTANCE):
            self.found_line = True
            return True
        return False

    def determine_orientation(self):
        if self.found_line and not self.aligned and self.slope >= self.MIN_SLOPE:
            self.aligned = True
            return True
        return False

    def update_history(self, x):
        self.line_history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def reset(self):
        self.line_history = [0] * self.BUFF_SIZE
        self.found_line = False
        self.aligned = False
        self.history_idx = 0
        self.slope = None

                                             
def main(args=None):
    rclpy.init(args=args)

    transform = TransformPublisher()

    rclpy.spin(transform)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    transform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

