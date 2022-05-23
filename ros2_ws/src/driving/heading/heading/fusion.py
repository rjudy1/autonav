################################
# AutoNav 2022 Competition Robot
# Package: heading
# File: fusion.py
# Purpose: detect potholes and remove back 180 of lidar sweep
# Date Modified: 22 May 2022
# Fuses encoder and gps headings
################################

# !/usr/bin/env python

import rclpy
from rclpy.node import Node
from utils.utils import *

from custom_msgs.msg import EncoderData
from custom_msgs.msg import HeadingStatus
from std_msgs.msg import String


class Fusion(Node):
    def __init__(self):
        super().__init__('lidar_modifier')

        self.declare_parameter("/InitialHeading", 0.000)
        self.declare_parameter('/EncoderWeight', .5)
        self.declare_parameter('/Debug', False)

        self.state = STATE.LINE_FOLLOWING
        self.curr_heading = self.get_parameter('/InitialHeading')
        self.target_heading = 0.0
        self.encoder_curr_heading = self.get_parameter('/InitialHeading')

        self.encoder_sub = self.create_subscription(EncoderData, "encoder_data", self.enc_callback, 10)
        self.gps_heading_sub = self.create_subscription(HeadingStatus, "gps_heading", self.gps_callback, 10)
        self.wheel_pub = self.create_publisher(String, "wheel_distance", 10)
        self.event_pub = self.create_publisher(String, "gps_events", 10)

    def state_callback(self, new_state):
        self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = int(new_state.data)

    # encoder must be sufficiently faster than GPS to be considered updated
    def enc_callback(self, enc_msg):
        # 64 cm gap between wheel centers
        self.encoder_curr_heading += (enc_msg.right - enc_msg.left) / .32
        if self.get_parameter('/Debug').value:
            self.get_logger().info(f'encoder report: {enc_msg}, {self.encoder_curr_heading}')

    def gps_callback(self, gps_msg):
        self.target_heading = gps_msg.target_heading
        self.curr_heading = (1-self.get_parameter('/EncoderWeight')) * gps_msg.curr_heading\
                             + self.get_parameter('/EncoderWeight') * self.encoder_curr_heading

        # calculate and filter the error in the angle of the two headings
        error_angle = self.sub_angles(self.target_heading, self.curr_heading)
        filtered_error_angle = self.filter_angle(error_angle)
        self.get_logger().warning("Current Heading: " + str(self.curr_heading) + '\n' +
                                  "Target Heading: " + str(self.target_heading) + '\n' +
                                  "Error: " + str(filtered_error_angle))

        # check state, if in object avoid and GPS, then check error_angle for release
        if self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:
            if self.is_object_clear(filtered_error_angle):
                self.get_logger().info(self.WAYPOINT_STRAIGHT)
                msg = String()
                msg.data = WAYPOINT_STRAIGHT
                self.event_pub.publish(msg)

        # publish
        msg = String()
        msg.data = GPS_SENDER + ',' + str(filtered_error_angle)
        self.wheel_pub.publish(msg)

        if self.get_parameter('/Debug').value:
            self.get_logger().info(f'encoder report: {gps_msg}, {self.curr_heading}')


def main(args=None):
    rclpy.init(args=args)
    fuser = Fusion()

    try:
        rclpy.spin(fuser)
    except Exception as e:
        # Destroy the node explicitly (optional)
        fuser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
