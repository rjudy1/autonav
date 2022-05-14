################################
# AutoNav 2022 Competition Robot
# Package: rs2l_transform
# File: transform.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 11 May 2022
# To run: ros2 run rs2l_transform transform
################################

# !/usr/bin/env python

import sys

sys.path.insert(1, '/home/autonav/autonav/')

from dataclasses import dataclass
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from utils import *

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


@dataclass
class Circle:
    xcenter: float
    ycenter: float
    radius: float


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('lidar_modifier')
        self.lidar_pub = self.create_publisher(LaserScan, '/mod_lidar', 10)
        self.lidar_str_pub = self.create_publisher(String, '/mod_lidar', 10)
        self.i = 0

        # Subscribe to the camera color image
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # line detection parameters
        self.history_idx = 0
        self.slope = None
        self.aligned = False
        self.found_line = False
        self.window_handle = []

        self.lidar_trim_min = 1.57
        self.lidar_trim_max = 4.71

        self.in_front_min = 15/180*math.pi
        self.in_front_max = 2*math.pi - 15/180*math.pi

        # Read ROS Params
        self.declare_parameter("/PotholeBufferSize", 5)
        self.declare_parameter("/PotholeBufferFill", 0.8)
        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value
        self.BUFF_FILL = self.get_parameter('/PotholeBufferFill').value

        self.circles = []

        # Subscribe to state updates for the robot
        # self.state_sub = self.create_subscription(String, "state_topic", self.state_callback)

        # Initialize primary variables
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True
        self.window_handle = []

        self.get_logger().info("Waiting for image topics...")

    # given ax+bx+c=0 and center and radius of a circle, determine if they intersect
    def check_collision(self, a, b, c, x, y, radius):

        # Finding the distance of line
        # from center.
        dist = ((abs(a * x + b * y + c)) /
                math.sqrt(a * a + b * b))

        # Checking if the distance is less
        # than, greater than or equal to radius.
        # return the distance from the circle edge (approximate) and whether it touches
        if radius < dist:
            return 0, False
        else:
            return math.sqrt((x - 190) * (x - 190) + (y + 182)(y + 182)) * 1.5 / 380 - .6096, True

    # edit lidar here including pothole modifications ----------------------------------------------------------
    # first portion nullifies all data behind the scanner after adjusting min and max to be 0
    def lidar_callback(self, scan):
        # adjust range
        scan.angle_max += abs(scan.angle_min)
        scan.angle_min = 0.0

        scan_range = scan.angle_max - scan.angle_min
        trim_range = self.lidar_trim_max - self.lidar_trim_min
        width = round(trim_range / scan_range * len(scan.ranges))

        shift = self.lidar_trim_min - scan.angle_min
        trim_base = round(shift / scan_range * len(scan.ranges))

        if len(scan.intensities) > trim_base + width:
            for i in range(trim_base, trim_base + width):
                scan.ranges[i] = math.inf
                scan.intensities[i] = 0.0
        else:
            for i in range(trim_base, trim_base + width):
                scan.ranges[i] = math.inf

        # insert pothole additions to lidar here
        for circle in self.circles:
            self.get_logger().info(circle)
            for i in range(len(scan.ranges) // 4):
                dist, hit = self.check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
                                                 190 * (math.cos(i * scan.angle_increment) - 182) + 182 * (
                                                            math.sin(i * scan.angle_increment) + 190),
                                                 circle.xcenter, circle.ycenter, circle.radius)[1]
                if hit:
                    scan.ranges[i] = dist
                    scan.intensities[i] = 47

            for i in range(3 * len(scan.ranges) // 4, len(scan.ranges)):
                dist, hit = self.check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
                                                 190 * (math.cos(i * scan.angle_increment) - 182) + 182 * (
                                                            math.sin(i * scan.angle_increment) + 190),
                                                 circle.xcenter, circle.ycenter, circle.radius)[1]
                if hit:
                    scan.ranges[i] = dist
                    scan.intensities[i] = 47

        self.lidar_pub.publish(scan)
        msg = String()
        msg = "PATH_CLEAR"
        # scan in the narrow range in front to check for obstacles
        for i in range(len(scan.intensities)):
            if scan.ranges[i] < 1.5 and (i*scan.angle_increment < self.in_front_min or i*scan.angle_increment > self.in_front_max):

                msg.data = "OBJECT_SEEN"
                self.history[self.history_idx] = 1 if 1.5 < self.DISTANCE_AT_WHICH_WE_STOP_FROM_THE_OBSTACLE else 0
                self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE
                break


        # self.get_logger().info('Publishing LaserScan ' + str(scan.header.stamp) + ' transformed:\n' + 'Angle_min: ' + str(scan.angle_min)
        #                             + '\nAngle_max: ' + str(scan.angle_max))

        if self.path_clear and np.count_nonzero(self.history) >= self.BUFF_FILL * self.BUFF_SIZE:
            self.get_logger().info("OBJECT_SEEN")
            self.lidar_str_pub.publish(msg)
            self.path_clear = False
        elif (self.state == STATES.LINE_TO_OBJECT or self.state == STATES.GPS_TO_OBJECT) and np.count_nonzero(self.history) <= (1 - self.BUFF_FILL) * self.BUFF_SIZE:
            self.get_logger().info("PATH_CLEAR")
            self.lidar_str_pub.publish(msg)
            self.path_clear = True

    def image_callback(self, image):
        # Bridge Image
        image = bridge_image(image, "bgr8")

        # Apply Blur
        # grey = cv2.medianBlur(image,3)

        # Apply HSV Filter
        gray = hsv_filter(image)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))

        # Find Circles
        circles = cv2.HoughCircles(morph, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=50, maxRadius=150)
        # clear circles if not found
        self.circles = []  # replace previously viewed circles in array

        # Display Circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # draw the outer circle
                cv2.circle(morph, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(morph, (i[0], i[1]), 2, (0, 0, 255), 3)
                self.circles.insert(0, Circle(i[0], i[1], i[2]))  # store all circles in the array

        # Display Result
        cvDisplay(morph, 'Pothole Detection', self.window_handle)

        if circles is not None: self.update_history(1)
        # self.determine_state()

    def update_history(self, x):
        self.history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def reset(self):
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.path_clear = True


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
