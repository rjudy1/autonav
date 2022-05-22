################################
# AutoNav 2022 Competition Robot
# Package: wheel_controller
# File: controller.py
# Purpose: control wheels
# Date Modified: 21 May 2022
################################

# !/usr/bin/env python

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import sys
sys.path.insert(1, '/home/autonav/autonav/')
from utils import *

RIGHT_WHEEL = 0x80
BAUD_RATE = 19200
PORT = "/dev/ttyUSB0"
CCW = 0x40


class Wheels:
    def __init__(self, port="/dev/ttyUSB0", BAUD=19200):
        self.serialPort = serial.Serial(port, BAUD)

    def __del__(self):
        self.serialPort.close()

    def control_wheels(self, left_speed, right_speed):
        # control right wheel
        right_cmd = self.convert_to_hex_cmd(-1*right_speed) | RIGHT_WHEEL
        self.serialPort.write(right_cmd)

        # control left wheel
        left_cmd = self.convert_to_hex_cmd(left_speed)
        self.serialPort.write(left_cmd)

        self.serialPort.flush()

    def convert_to_hex_cmd(self, speed):
        # get magnitude of speed and direction
        cmd = abs(speed)
        if speed < 0:
            cmd |= CCW
        return cmd

class WheelControl(Node):
    def __init__(self):
        super.__init__('wheels_controller')
        self.wheel_sub = self.create_subscriber(String, "wheel_distance", wheel_callback, 10)

        # start in a stopped state
        self.curr_right_speed = 0
        self.curr_left_speed = 0
        self.boost_count = 0

        self.declare_parameter('/FollowingDirection')
        self.declare_parameter('/LineDist')
        self.declare_parameter('/SideObjectDist')
        self.declare_parameter('/DefaultSpeed',15)
        self.declare_parameter('/BoostIncrease',1)
        self.declare_parameter('/BoostCountThreshold',20)
        self.declare_parameter('/LineBoostMargin',30.0)
        self.declare_parameter('/GPSBoostMargin',.1745)

        self.following_direction = self.get_parameter('/FollowingDirection')
        self.target_line_dist = self.get_parameter('/LineDist').value
        self.target_obj_dist = self.get_parameter('/SideObjectDist').value
        self.default_speed = self.get_parameter('/DefaultSpeed').value
        self.speed_boost = self.get_parameter('/BoostIncrease').value
        self.boost_count_threshold = self.get_parameter('/BoostCountThreshold').value
        self.line_boost_margin = self.get_parameter('/LineBoostMargin').value
        self.gps_boost_margin = self.get_parameter('/GPSBoostMargin').value

        # self.pid_line = PIDController()  # for lie following
        # self.pid_obj = PIDController()   # for object avoidance
        # self.pid_gps = PIDController()   # for during gps navigation

        self.driver = Wheels()
        self.following_mode = FollowMode.eeNone


    def __del__(self):
        pass

    def start_control(self):
        pass

    def signal_catch(self, signum):
        pass

    def wheel_calllback(self, msg):
        left_speed = self.curr_left_speed
        right_speed = self.curr_right_speed

        stop_override = False
        message_valid = False

        self.get_logger().info(f"received msg {msg}")

        # put all this code in place
        if msg[:3] == WHEELS_OBJECT_AVOIDANCE:
            self.following_mode = FollowMode.eeObject
            self.boostCount = 0
            self.get_logger().info("SWITCHED TO OBJECT AVOIDANCE")
        elif msg[:3] == WHEELS_LINE_FOLLOWING:
            pass
        elif msg[:3] == WHEELS_GPS_NAV:
            pass
        elif msg[:3] == WHEELS_TRANSITION:
            pass
        elif msg[:3] == TRANSITION_CODE:
            pass
        elif self.following_mode==FollowMode.eeLine and msg[:3]==LIN_SENDER:
            pass
        elif self.following_mode == FollowMode.eeObject && msg[:3]==OBJECT_SENDER:
            pass
        elif self.following_mode == FollowMode.eeGps and msg[:3] == GPS_SENDER:
            pass

        self.get_logger().info(f"Calculated left and right: {left_speed} and {right_speed}")
        if self.boost_count > self.boost_count_threshold and message_valid:
            left_speed += self.speed_boost
            right_speed += self.speed_boost

        if left_speed != self.curr_left_speed or self.curr_right_speed != right_speed:
            if not stop_override:
                MAX_CHANGE = abs(MAX_CHANGE) if left_speed > self.curr_left_speed + MAX_CHANGE else 0-MAX_CHANGE
                left_speed = self.curr_left_speed + MAX_CHANGE
                right_speed = self.curr_right_speed + MAX_CHANGE

            self.driver.control_wheels(left_speed, right_speed)

        self.curr_left_speed = left_speed
        self.curr_right_speed = right_speed


def main(args=None):
    rclpy.init(args=args)

    wheels = WheelControl()

    rclpy.spin(wheels)

    # Destroy the node explicitly (optional)
    wheels.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
