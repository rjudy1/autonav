#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Top-Level ROS Node Class                      #
# ********************************************* #

import sys
sys.path.insert(1, '/home/autonav/autonav_ws/src/autonav_utils')

import cv2
import rospy
from autonav_class import AutoNavClass
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class AutoNavNode(AutoNavClass):

    def __init__(self, name):
        self.node_name = name
        rospy.init_node(self.node_name)
        
        # States
        self.LINE_FOLLOWING = "LINE_FOLLOWING_STATE"
        self.OBJECT_AVOIDANCE_FROM_LINE = "OBJECT_AVOIDANCE_FROM_LINE_STATE"
        self.OBJECT_AVOIDANCE_FROM_GPS = "OBJECT_AVOIDANCE_FROM_GPS_STATE"
        self.GPS_NAVIGATION = "GPS_NAVIGATION_STATE"
        self.LINE_TO_OBJECT = "LINE_TO_OBJECT_STATE"
        self.OBJECT_TO_LINE = "OBJECT_TO_LINE"
        self.GPS_TO_OBJECT = "GPS_TO_OBJECT"
        self.FIND_LINE = "FIND_LINE"
        self.LINE_ORIENT = "LINE_ORIENT"

        # start in line detection
        self.state = self.OBJECT_AVOIDANCE_FROM_LINE

        # start in line following
        #self.state = self.LINE_FOLLOWING

        # Read ROS Params
        self.read_param("/FollowingDirection", "following_dir", 1)
        self.read_param("/Debug", "is_debug", False)
        self.read_param("/DanielDebug", "DANIEL_DEBUG", False)
        self.read_param("/IsaiahDebug", "ISAIAH_DEBUG", False)
        self.read_param("/JoshuaDebug", "JOSHUA_DEBUG", False)
        self.read_param("/TrevorDebug", "TREVOR_DEBUG", False)

        # Display window variable
        self.window_handle = []

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

    def state_callback(self, new_state):
        rospy.loginfo("New State Received ({}): {}".format(self.node_name, new_state.data))
        self.state = new_state.data

    # Use cv_bridge() to convert the ROS image to OpenCV format
    def bridge_image(self, ros_image, format):
        try: cv_image = self.bridge.imgmsg_to_cv2(ros_image, format)
        except CvBridgeError as e: rospy.logerr("CvBridge could not convert images from ROS to OpenCV")
        return cv_image

    def bridge_image_pub(self, cv_image, format):
        try: ros_image = self.bridge.cv2_to_imgmsg(cv_image, format)
        except CvBridgeError as e: rospy.logerr("CvBridge could not convert images from OpenCV to ROS")
        return ros_image

    # Read param from params.yaml and set var_name to value from file or default_val
    def read_param(self, param, var_name, default_val):
        if rospy.has_param(param):
            setattr(self, var_name, rospy.get_param(param))
        else:
            rospy.logwarn("%s not found. Using default instead. (%s)" % (param, default_val))
            setattr(self, var_name, default_val)

    def cleanup(self):
        rospy.loginfo("Shutting down " + self.node_name + " node")
        cv2.destroyAllWindows()
    
    def safe_publish(self, publisher, data):
        try: publisher.publish(data)
        except rospy.ROSException as e: pass
