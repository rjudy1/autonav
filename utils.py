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
import rclpy


class STATES:
    LINE_FOLLOWING = "LINE_FOLLOWING_STATE"
    OBJECT_AVOIDANCE_FROM_LINE = "OBJECT_AVOIDANCE_FROM_LINE_STATE"
    OBJECT_AVOIDANCE_FROM_GPS = "OBJECT_AVOIDANCE_FROM_GPS_STATE"
    GPS_NAVIGATION = "GPS_NAVIGATION_STATE"
    LINE_TO_OBJECT = "LINE_TO_OBJECT_STATE"
    OBJECT_TO_LINE = "OBJECT_TO_LINE"
    GPS_TO_OBJECT = "GPS_TO_OBJECT"
    FIND_LINE = "FIND_LINE"
    LINE_ORIENT = "LINE_ORIENT"

def hsv_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 65, 100])
    upper = np.array([179, 255, 255])
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
def closeWindows(handleArr):
    if len(handleArr) > 0:
        for handle in handleArr:
            cv2.destroyWindow(handle)
            handleArr.remove(handle)


# Closes unwanted windows
def cvDisplay(image, handle, handleArr):
    cv2.imshow(handle, image)
    cv2.waitKey(2)
    if handle not in handleArr:
        handleArr.append(handle)