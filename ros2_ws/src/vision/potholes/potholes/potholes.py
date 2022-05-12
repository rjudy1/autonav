#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Pothold Detection Class                       #
# ********************************************* #

import sys
sys.path.insert(1, '/home/autonav/autonav/')

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils import *

from std_msgs.msg import String

# Start RealSense Publisher: $ rosrun publish publish.py
# Run This Script: $ rosrun potholes potholes.py

class PotholeDetection(Node):
    def __init__(self):
        super().__init__("potholes")

        # States
        self.POTHOLE_SEEN = "POTHOLE_SEEN"
        self.POTHOLE_CLEAR = "POTHOLE_CLEAR"

        # Read ROS Params
        self.declare_parameter("/PotholeBufferSize", 5)
        self.declare_parameter("/PotholeBufferFill", 0.8)
        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value
        self.BUFF_FILL = self.get_parameter('/PotholeBufferFill').value

        # Publish events that could change the robot state
        self.event_pub = self.create_publisher(String, "pothole_events", 10)

        # Subscribe to the camera color topic
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback,10)

        # Subscribe to state updates for the robot
        # self.state_sub = self.create_subscription(String, "state_topic", self.state_callback)

        # Initialize primary variables
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True
        self.window_handle = []

        self.get_logger().info("Waiting for image topics...")

    def update_history(self, x):
        self.history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def determine_state(self):
        if self.path_clear and np.count_nonzero(self.history) >= self.BUFF_FILL * self.BUFF_SIZE:
            self.get_logger().info(self.POTHOLE_SEEN)

#            self.safe_publish(self.event_pub, self.POTHOLE_SEEN)
            self.path_clear = False
        elif (self.state in {self.LINE_TO_OBJECT, self.GPS_TO_OBJECT}) and np.count_nonzero(self.history) <= (1 - self.BUFF_FILL) * self.BUFF_SIZE:
#            rospy.loginfo(self.POTHOLE_CLEAR)
#            self.safe_publish(self.event_pub, self.POTHOLE_CLEAR)
            self.path_clear = True

    def reset(self):
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.path_clear = True

    def image_callback(self, ros_image):

        # Bridge Image
        image = bridge_image(ros_image, "bgr8")

        # Apply Blur
        #grey = cv2.medianBlur(image,3)

        # Apply HSV Filter
        gray = hsv_filter(image)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))

        # Find Circles
        circles = cv2.HoughCircles(morph, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=50, maxRadius=150)

        # Display Circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(morph,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(morph,(i[0],i[1]),2,(0,0,255),3)

        # Display Result
        cvDisplay(morph, 'Pothole Detection', self.window_handle)

        if circles is not None: self.update_history(1)
        # self.determine_state()


def main():
    rclpy.init()
    pd = PotholeDetection()

    try: rclpy.spin(pd)
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
