################################
# AutoNav 2022 Competition Robot
# Package: heading
# File: fusion.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 22 May 2022
# Fuses encoder and gps headings
################################

# !/usr/bin/env python
import cmath
import math

import rclpy
from rclpy.node import Node
from utils.utils import *

from custom_msgs.msg import EncoderData
from custom_msgs.msg import HeadingStatus
from std_msgs.msg import String


class Fusion(Node):
    def __init__(self):
        super().__init__('fusion')

        self.declare_parameter("/InitialHeading", 0.000)
        self.declare_parameter('/EncoderWeight', .5)
        self.declare_parameter('/Debug', False)

        self.declare_parameter('/ExitAngle', .2)
        self.exit_angle = self.get_parameter('/ExitAngle').value
        self.state = STATE.LINE_FOLLOWING
        self.curr_heading = self.get_parameter('/InitialHeading').value
        self.target_heading = 0.0
        self.encoder_curr_heading = self.get_parameter('/InitialHeading').value
        self.moving_avg = np.zeros((5,), dtype=np.float32)
        self.moving_avg_idx = 0

        self.encoder_sub = self.create_subscription(EncoderData, "encoder_data", self.enc_callback, 10)
        self.gps_heading_sub = self.create_subscription(HeadingStatus, "gps_heading", self.gps_callback, 10)
        self.wheel_pub = self.create_publisher(String, "wheel_distance", 10)
        self.event_pub = self.create_publisher(String, "gps_events", 10)

    def state_callback(self, new_state):
        # self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = int(new_state.data)

    # encoder must be sufficiently faster than GPS to be considered updated
    def enc_callback(self, enc_msg):
        # 64 cm gap between wheel centers
        self.encoder_curr_heading = ((enc_msg.right - enc_msg.left) / .32) % (math.pi * 2)
        if self.encoder_curr_heading > math.pi:
            self.encoder_curr_heading += -2*math.pi
        if self.get_parameter('/Debug').value:
            pass
#            self.get_logger().info(f'encoder report: {enc_msg}, {self.encoder_curr_heading}')

    # a 5 point moving average filter
    def filter_angle(self, new_val):
        self.moving_avg = np.roll(self.moving_avg, 1)
        self.moving_avg[0] = new_val
        return sum(self.moving_avg) / self.moving_avg.size

    # subtracts the angles (x - y) and gives the answer between (-pi, pi]
    def sub_angles(self, x, y):
        a = (x - y + 2 * cmath.pi) % (2 * cmath.pi)
        if a > cmath.pi:
            a -= 2 * cmath.pi
        return a

    # return true if we can switch back from obstacle avoidance to GPS nav
    def is_heading_restored(self, error_angle):
        return abs(error_angle) < self.exit_angle

    def gps_callback(self, gps_msg):
        if -0.01 < gps_msg.current_heading < 0.01:
            encoder_weight = 1.0
        else:
            encoder_weight = self.get_parameter('/EncoderWeight').value
        # self.get_logger().info(f"GPS HEADING: {gps_msg}, encoder heading: {self.encoder_curr_heading}")
        self.target_heading = gps_msg.target_heading
        self.curr_heading = (1-encoder_weight) * gps_msg.current_heading + encoder_weight * self.encoder_curr_heading

        # calculate and filter the error in the angle of the two headings
        error_angle = self.sub_angles(self.target_heading, self.curr_heading)
        # filtered_error_angle = self.filter_angle(error_angle)
        if self.get_parameter('/Debug').value:
            self.get_logger().warning("Current GPS HEADING: " + str(gps_msg.current_heading) + '\n' +
                                      "Current Encoder Heading: " + str(self.encoder_curr_heading) + '\n' +
                                      "Current Weighted Heading " + str(self.curr_heading) + '\n' +
                                      "Target Heading: " + str(gps_msg.target_heading) + '\n' +
                                      "Error: " + str(error_angle))

        # check state, if in object avoid and GPS, then check error_angle for release
        if self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS or STATE.OBJECT_AVOIDANCE_FROM_LINE:
            if self.is_heading_restored(error_angle) and self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
                # self.get_logger().info(self.WAYPOINT_STRAIGHT)
                msg = String()
                msg.data = STATUS.HEADING_RESTORED
                self.event_pub.publish(msg)

        # publish
        msg = String()
        msg.data = CODE.GPS_SENDER + ',' + str(error_angle)
        # self.get_logger().warning(f"SENDING GPS ERROR {error_angle}")
        self.wheel_pub.publish(msg)

        # if self.get_parameter('/Debug').value:
        #     self.get_logger().info(f'encoder report: {gps_msg}, {self.curr_heading}')


def main(args=None):
    rclpy.init(args=args)
    fuser = Fusion()

    try:
        rclpy.spin(fuser)
    except KeyboardInterrupt:
        # Destroy the node explicitly (optional)
        fuser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
