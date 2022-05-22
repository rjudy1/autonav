################################
# AutoNav 2022 Competition Robot
# File: utils.py
# Purpose: store the different states used in the state machine
# Author: Rachael Judy
# Date Modified: 12 May 2022
################################

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class STATE:
    LINE_FOLLOWING = 0
    OBJECT_AVOIDANCE_FROM_LINE = 1
    OBJECT_AVOIDANCE_FROM_GPS = 2
    GPS_NAVIGATION = 3
    LINE_TO_OBJECT = 4
    OBJECT_TO_LINE = 5
    GPS_TO_OBJECT = 6
    FIND_LINE = 7
    LINE_ORIENT = 8


class DIRECTION:
    LEFT = 0
    RIGHT = 1


class FollowMode:
    eeNone = 0
    eeLine = 1
    eeObject = 2
    eeGps = 3
    eeTransition = 4


# Messages that indicate a change of state is needed
PATH_CLEAR = "PATH_CLEAR"
OBJECT_SEEN = "OBJECT_SEEN"
WAYPOINT_FOUND = "WAYPOINT_FOUND"
WAYPOINT_STRAIGHT = "WAYPOINT_STRAIGHT"
FOUND_LINE = "FOUND_LINE"
ALIGNED = "ALIGNED"

# Messages that change the wheel controller's state
WHEELS_TRANSITION = "STR"
WHEELS_OBJECT_AVOIDANCE = "SOA"
WHEELS_LINE_FOLLOWING = "SLF"
WHEELS_GPS_NAV = "SGN"
STOP_CODE = "STO"
TRANSITION_CODE = "TRA"
LIN_SENDER = "LIN"
OBJECT_SENDER = "OBJ"
GPS_SENDER = "GPS"
TURN_SPEED = 14
SLIGHT_TURN = 10
STOP_CODE = 7777


def hsv_filter(image, use_white=True):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if not use_white:
        lower = np.array([0, 65, 100])
        upper = np.array([179, 255, 255])
    else:  # for white line following
        lower = np.array([0, 0, 170])
        upper = np.array([179, 70, 255])
    return cv2.inRange(hsv, lower, upper)


# Use cv_bridge() to convert the ROS image to OpenCV format
def bridge_image(ros_image, format):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, format)
        return cv_image
    except CvBridgeError as e:
        print("CvBridge could not convert images from ROS to OpenCV")


def bridge_image_pub(cv_image, format):
    try:
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, format)
        return ros_image
    except CvBridgeError as e:
        print("CvBridge could not convert images from OpenCV to ROS")


def cleanup():
    cv2.destroyAllWindows()


# Function used to display images in a way that allows them to be closed when not used.
def close_windows(handleArr):
    if len(handleArr) > 0:
        for handle in handleArr:
            cv2.destroyWindow(handle)
            handleArr.remove(handle)


# Closes unwanted windows
def cv_display(image, handle, handleArr):
    cv2.imshow(handle, image)
    cv2.waitKey(2)
    if handle not in handleArr:
        handleArr.append(handle)
