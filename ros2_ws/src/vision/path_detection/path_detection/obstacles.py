################################
# AutoNav 2022 Competition Robot
# Package: rs2l_transform
# File: obstacles.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 24 May 2022
# To run: ros2 run path_detection obstacles
################################

# !/usr/bin/env python

from dataclasses import dataclass
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from utils.utils import *
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


@dataclass
class Circle:
    xcenter: float
    ycenter: float
    radius: float


# given ax+bx+c=0 and center and radius of a circle, determine if they intersect and returns distance in meters from edge
def check_collision(a, b, c, x, y, radius):
    # Finding the distance of line from center
    dist = ((abs(a * x + b * y + c)) / math.sqrt(a * a + b * b))

    # return the distance from the circle edge (approximate) and whether it touches
    if radius < dist:
        return 0, False
    else:
        return math.sqrt((x - 190) * (x - 190) + (y - 452) * (y - 452)) / 150 - 0.3, True


class TransformPublisher(Node):
    def __init__(self):
        super().__init__('lidar_modifier')
        self.lidar_pub = self.create_publisher(LaserScan, '/laser_frame', 10)
        self.lidar_str_pub = self.create_publisher(String, '/mod_lidar', 10)
        self.lidar_wheel_distance_pub = self.create_publisher(String, "wheel_distance", 10)

        # Subscribe to the camera color image and unaltered laser scan
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Subscribe to state updates for the robot
        self.state_sub = self.create_subscription(Int32, "state_topic", self.state_callback, 10)
        self.state = STATE.LINE_FOLLOWING

        # lidar parameters
        self.declare_parameter("/LIDARTrimMin", 1.57)
        self.declare_parameter("/LIDARTrimMax", 4.71)
        self.declare_parameter("/ObstacleFOV", math.pi/6)
        self.declare_parameter("/ObstacleDetectDistance", 1.5)  # meters
        self.declare_parameter("/FollowingDirection", 1)
        self.declare_parameter("/ObjectDistance", 4.0)

        # camera parameters
        self.declare_parameter('/LineDetectCropTop', 0.0)
        self.declare_parameter('/LineDetectCropBottom', 0.2)
        self.declare_parameter('/LineDetectCropSide', 0.2)
        self.declare_parameter("/PotholeBufferSize", 5)

        self.declare_parameter('/Debug', False)

        self.BUFF_SIZE = self.get_parameter('/PotholeBufferSize').value

        # camera/obstacle detection
        self.circles = []  # recent circle history
        self.window_handle = []
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.history_idx = 0
        self.path_clear = True

        self.get_logger().info("Waiting for image/lidar topics...")

    def state_callback(self, new_state):
        self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = new_state.data

    def get_c(self, i, scan):
        return -(190 * (452-math.cos(i * scan.angle_increment)) - 452 * (190-math.sin(i * scan.angle_increment)))

    # first portion nullifies all data behind the scanner after adjusting min and max to be 0
    # second portion adds potholes based on image data
    # third portion replaces obstacle in front and time of flight sensors
    def lidar_callback(self, scan):
        # adjust range to only include data in front of scanner
        scan.angle_max += abs(scan.angle_min)
        scan.angle_min = 0.0

        scan_range = scan.angle_max - scan.angle_min
        trim_range = self.get_parameter('/LIDARTrimMax').value - self.get_parameter('/LIDARTrimMin').value
        width = round(trim_range / scan_range * len(scan.ranges))

        shift = self.get_parameter('/LIDARTrimMin').value - scan.angle_min
        trim_base = round(shift / scan_range * len(scan.ranges))

        try:
            if len(scan.intensities) > trim_base + width:
                for i in range(trim_base, trim_base + width):
                    scan.ranges[i] = math.inf
                    scan.intensities[i] = 0.0
            else:
                for i in range(trim_base, trim_base + width):
                    scan.ranges[i] = math.inf
        except Exception:
            self.get_logger().info(f"ERROR: removing extraneous data broke ranges length: {len(scan.ranges)}")

        # insert pothole additions to lidar here - can compensate with constants for the camera angle
        for circle in self.circles:
            # front part of lidar scan 0 to pi/2 radians
            for i in range(len(scan.ranges)):
                if i < (len(scan.ranges) // 4) or i > len(scan.ranges) // 4 * 3:
                    dist, hit = check_collision(-math.cos(i * scan.angle_increment), math.sin(i * scan.angle_increment),
                                                self.get_c(i, scan), circle.xcenter, circle.ycenter, circle.radius)
                    if dist < scan.ranges[i] and hit:
                        scan.ranges[i] = dist
                        scan.intensities[i] = 47

        self.lidar_pub.publish(scan)

        # scan in the range in front of robot to check for obstacles
        msg = String()
        for i in range(len(scan.intensities)):
            if i * scan.angle_increment < self.get_parameter('/ObstacleFOV').value/2 \
                    or i * scan.angle_increment > 2*math.pi-self.get_parameter('/ObstacleFOV').value / 2:
                msg.data = STATUS.OBJECT_SEEN if self.get_parameter("/ObstacleDetectDistance").value > scan.ranges[i] \
                    else STATUS.PATH_CLEAR
                self.history[self.history_idx] = \
                    1 if self.get_parameter("/ObstacleDetectDistance").value > scan.ranges[i] else 0
                self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE
                if msg.data == STATUS.OBJECT_SEEN:  break

        if self.path_clear and np.count_nonzero(self.history) >= 0.6 * self.BUFF_SIZE:
            self.get_logger().info("OBJECT_SEEN")
            self.lidar_str_pub.publish(msg)
            self.path_clear = False
        elif (self.state == STATE.LINE_TO_OBJECT or self.state == STATE.GPS_TO_OBJECT) \
                and np.count_nonzero(self.history) <= (1 - .6) * self.BUFF_SIZE:
            self.get_logger().info("PATH_CLEAR")
            self.lidar_str_pub.publish(msg)
            self.path_clear = True

        # publish the wheel distance from the obstacle based on following direction
        distance_msg = String()
        try:
            if self.get_parameter('/FollowingDirection').value == DIRECTION.LEFT \
                and scan.ranges[round(math.pi * .375 / scan.angle_increment)] is not None \
                and scan.ranges[round(math.pi * .375 / scan.angle_increment)] < 5.0:
                distance_msg.data = "OBJ," + str(scan.ranges[round(math.pi * .375 / scan.angle_increment)])
                self.lidar_wheel_distance_pub.publish(distance_msg)
                if self.get_parameter('/Debug').value:
                    # self.get_logger().info(
                    #     f"Direction: {self.get_parameter('/FollowingDirection').value} : {distance_msg.data}")
                    pass

            elif self.get_parameter('/FollowingDirection').value == DIRECTION.RIGHT \
                    and scan.ranges[round(math.pi * 1.625 / scan.angle_increment)] is not None \
                    and scan.ranges[round(math.pi * 1.625 / scan.angle_increment)] < 5:
                distance_msg.data = "OBJ," + str(scan.ranges[round(math.pi * 1.625 / scan.angle_increment)])
                self.lidar_wheel_distance_pub.publish(distance_msg)
                if self.get_parameter('/Debug').value:
                    # self.get_logger().info(
                    #     f"Following direction: {self.get_parameter('/FollowingDirection').value}: {distance_msg.data}")
                    pass
        except Exception as e:
            self.get_logger().warning(f"ERROR with TOF Following direction : {e} : {distance_msg.data} m")

    # receives camera image and parses potholes into history
    def image_callback(self, image):
        image = bridge_image(image, "bgr8")
        # slice edges
        y, x = image.shape[0], image.shape[1]
        image = image[int(y * self.get_parameter('/LineDetectCropTop').value):-int(y * self.get_parameter('/LineDetectCropBottom').value),
                int(x * self.get_parameter('/LineDetectCropSide').value):-int(x * self.get_parameter('/LineDetectCropSide').value)]

        # Apply HSV Filter
        gray = hsv_filter(image)
        morph = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # look for certain type blobs aka potholes - hone these with other obstacles
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
        keypoints = detector.detect(morph)  # find the blobs meeting the parameters
        self.circles = []
        for hole in keypoints:
            self.circles.insert(0, Circle(hole.pt[0], hole.pt[1], hole.size//2))

        if self.get_parameter('/Debug').value:
            blobs = cv2.drawKeypoints(morph, keypoints, np.zeros((1, 1)), (0, 255, 0),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv_display(blobs, 'Potholes', self.window_handle)

    def update_history(self, x):
        self.history[self.history_idx] = x
        self.history_idx = (self.history_idx + 1) % self.BUFF_SIZE

    def reset(self):
        self.history = np.zeros((self.BUFF_SIZE,), dtype=bool)
        self.path_clear = True


def main(args=None):
    rclpy.init(args=args)

    transform = TransformPublisher()

    try:
        rclpy.spin(transform)
    except KeyboardInterrupt:
        # Destroy the node explicitly (optional)
        transform.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
