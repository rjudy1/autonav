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

import cv2

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
        self.lidar_pub = self.create_publisher(LaserScan, '/laser_frame', 10)
        self.lidar_str_pub = self.create_publisher(String, '/mod_lidar', 10)
        self.lidar_wheel_distance_pub = self.create_publisher(String, "wheel_distance", 10)
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

        self.declare_parameter("/LIDAR_Trim_Min", 1.57)
        self.declare_parameter("/LIDAR_Trim_Max", 4.71)

        self.in_front_min = 15/180*math.pi
        self.in_front_max = 2*math.pi - 15/180*math.pi

        # Read ROS Params
        self.declare_parameter("/PotholeBufferSize", 5)
        self.declare_parameter("/PotholeBufferFill", 0.8)
        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value
        self.BUFF_FILL = self.get_parameter('/PotholeBufferFill').value

        self.declare_parameter('/Debug', False)

        self.circles = []

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(String, "state_topic", self.state_callback, 10)
        self.state = STATES.LINE_FOLLOWING
        # Initialize primary variables
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True
        self.window_handle = []
        self.obstacle_detect_distance = 1.5

        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)

        self.declare_parameter("/FollowingDirection", 1)
        self.FOLLOWING_DIR = self.get_parameter('/FollowingDirection').value
        self.CROP_TOP = self.get_parameter('/LineDetectCropTop').value
        self.CROP_BOTTOM = self.get_parameter('/LineDetectCropBottom').value
        self.CROP_SIDE = self.get_parameter('/LineDetectCropSide').value

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
            return math.sqrt((x - 190) * (x - 190) + (y + 182) * (y + 182)) * 1.5 / 380 - .6096, True

    def state_callback(self, new_state):
        # self.get_logger().info("New State Received ({}): {}".format(self.node_name, new_state.data))
        self.state = new_state.data


    # edit lidar here including pothole modifications ----------------------------------------------------------
    # first portion nullifies all data behind the scanner after adjusting min and max to be 0
    def lidar_callback(self, scan):
        # adjust range
        scan.angle_max += abs(scan.angle_min)
        scan.angle_min = 0.0

        scan_range = scan.angle_max - scan.angle_min
        trim_range = self.get_parameter('/LIDAR_Trim_Max').value - self.get_parameter('/LIDAR_Trim_Min').value
        width = round(trim_range / scan_range * len(scan.ranges))

            # self.get_logger().warning(f"{}")

        shift = self.get_parameter('/LIDAR_Trim_Min').value - scan.angle_min
        trim_base = round(shift / scan_range * len(scan.ranges))
        self.get_logger().warning(f"\n{len(scan.ranges)}, {round(trim_base)}, {round(width)}")

        if len(scan.intensities) > trim_base + width:
            for i in range(trim_base, trim_base + width):
                scan.ranges[i] = math.inf
                scan.intensities[i] = 0.0

        else:
            try:
                for i in range(trim_base, trim_base + width):
                    scan.ranges[i] = math.inf
            except Exception:

                self.get_logger().info(f"ranges length: {len(scan.ranges)}")

        self.get_logger().info(f"\nranges length: {len(scan.ranges)}")

        # insert pothole additions to lidar here
        for circle in self.circles:
            # self.get_logger().info(circle)
            for i in range(len(scan.ranges) // 4):
                dist, hit = self.check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
                                                 190 * (math.cos(i * scan.angle_increment) - 182) + 182 * (
                                                            math.sin(i * scan.angle_increment) + 190),
                                                 circle.xcenter, circle.ycenter, circle.radius)
                if hit:
                    scan.ranges[i] = dist
                    scan.intensities[i] = 47

            for i in range(3 * len(scan.ranges) // 4, len(scan.ranges)):
                dist, hit = self.check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
                                                 190 * (math.cos(i * scan.angle_increment) - 182) + 182 * (
                                                            math.sin(i * scan.angle_increment) + 190),
                                                 circle.xcenter, circle.ycenter, circle.radius)
                if hit:
                    scan.ranges[i] = dist
                    scan.intensities[i] = 47

        self.lidar_pub.publish(scan)

        msg = String()
        msg.data = "PATH_CLEAR"
        # scan in the narrow range in front to check for obstacles
        for i in range(len(scan.intensities)):
            if scan.ranges[i] < 1.5 and (i*scan.angle_increment < self.in_front_min or i*scan.angle_increment > self.in_front_max):

                msg.data = "OBJECT_SEEN"
                self.history[self.history_idx] = 1 if 1.5 < self.obstacle_detect_distance else 0
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

        distance_msg = String()
        try:
            if self.FOLLOWING_DIR == 1 and scan.ranges[round(math.pi*13/8/scan.angle_increment)] != math.inf:
                    distance_msg.data = "OBJ," + str(scan.ranges[round(math.pi*13/8/scan.angle_increment)])
                    # self.get_logger().info(f"Following direction right? {self.FOLLOWING_DIR}: {distance_msg.data}")
            elif scan.ranges[round(math.pi*13/8/scan.angle_increment)] != math.inf:
                distance_msg.data = "OBJ," + str(scan.ranges[round(3*math.pi/8/scan.angle_increment)])
                # self.get_logger().info(f"Following direction left? {self.FOLLOWING_DIR}: {distance_msg.data}")
        except IndexError:
            pass
        #     self.get_logger().info(f"position in array {round(math.pi * 13 / 8 / scan.angle_increment)}")
        #     self.get_logger().info(f"ranges {scan.ranges}")

        self.lidar_wheel_distance_pub.publish(distance_msg)

    def image_callback(self, image):
        image = bridge_image(image, "bgr8")
        # slice edges
        y, x = image.shape[0], image.shape[1]
        image = image[int(y*self.CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]

        # Apply HSV Filter
        gray = hsv_filter(image)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))

        # find potholes
        # look for certain type blobs - hone these with other obstacles
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True  # area of acceptable blob
        params.minArea = 8000
        params.maxArea = 30000
        params.filterByCircularity = True  # square has circularity of like 78%
        params.minCircularity = .75
        params.maxCircularity = 1
        params.filterByConvexity = True  # more convexity is closer to circle
        params.minConvexity = .85
        params.maxConvexity = 1
        params.filterByInertia = True
        params.minInertiaRatio = .45
        params.maxInertiaRatio = 1
        params.filterByColor = False

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(morph)

        if len(keypoints) != 0:
            blank = np.zeros((1, 1))
            blobs = cv2.drawKeypoints(morph, keypoints, blank, (0, 255, 0),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cvDisplay(blobs, 'Potholes', self.window_handle)
            for hole in keypoints:
                self.circles.insert(0, Circle(hole.pt[0], hole.pt[1], hole.size//2))
            self.update_history(1)

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
