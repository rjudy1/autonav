#!/usr/bin/env python

################################
# Cedarville AutoNav 2022 Competition Robot
# Package: lines
# File: line_detection.py
# Purpose: controls line detection
# Author: Modified from 2020-21 autonav team code for ROS2
# Date Modified: 12 May 2022
################################
import sys
sys.path.insert(1, '/home/autonav/autonav/')

import cv2
import math
import numpy as np
import rclpy
from rclpy.node import Node
from utils import *


class LineDetection(Node):
    def __init__(self):
        super().__init__('detection')
        self.history_idx = 0
        self.slope = None
        self.aligned = False
        self.found_line = False
        self.window_handle = []

        # Read ROS Params - Line Detection
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


    def image_callback(self, image, state):
        # Save Dimensions
        y, x = image.shape[0], image.shape[1]

        # Slice Edges
        image = image[int(y*self.CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]

        cvDisplay(image, 'Line Detection Color Image', self.window_handle)

        # Remove Shadows
        image = hsv_filter(image)

        # Discard Oversaturated Images
        if np.count_nonzero(image) < image.shape[0]*image.shape[1]*self.MAX_WHITE:
            # Open Image
            morph = cv2.morphologyEx(image, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
            line, coords = self.determine_line(morph, state)
            if self.DEBUG_MODE:
                if line:
                    x1, y1, x2, y2 = coords
                    morph = cv2.cvtColor(morph, cv2.COLOR_GRAY2RGB)
                    cv2.line(morph, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)
                cvDisplay(morph, 'Line Detection Opened Image', self.window_handle)
        else:
            self.get_logger().warning("Line Detection Image Discarded")
            self.update_history(0)

        found_line = self.determine_state()
        aligned = self.determine_orientation()

        return found_line, aligned

    def determine_line(self, image, state):
        if state == "OBJECT_TO_LINE":
            lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH/2), maxLineGap=10)
        else:
            lines = cv2.HoughLinesP(image, 1, np.pi/180, 150, minLineLength=int(image.shape[0]*self.MIN_LINE_LENGTH), maxLineGap=10)

        if lines is not None:
            self.update_history(1)
            x1, y1, x2, y2 = lines[0][0]
            self.slope = self.get_slope(x1, y1, x2, y2)
            self.distance = self.get_distance(image.shape[1], image.shape[0], [x1, y1, x2, y2])
            self.get_logger.loginfo("LINE SLOPE: {} | LINE DISTANCE: {}".format(self.slope, self.distance))
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
