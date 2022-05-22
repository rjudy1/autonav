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
        cmd = hex(abs(speed))
        if speed < 0:
            cmd = cmd | CCW
        return cmd

class WheelControl(Node):
    def __init__(self):
        super.__init__('wheels_controller')
        self.wheel_sub = self.create_subscriber(String, "wheel_distance", self.wheel_callback, 10)
        self.unitChange = 1000 #assuming passed in meters, need mm

        # start in a stopped state
        self.curr_right_speed = 0
        self.curr_left_speed = 0
        self.boost_count = 0

        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/LineDist', 0.175)
        self.declare_parameter('/SideObjectDist', 0.6)
        self.declare_parameter('/DefaultSpeed', 15)
        self.declare_parameter('/BoostIncrease', 1)
        self.declare_parameter('/BoostCountThreshold', 20)
        self.declare_parameter('/LineBoostMargin', 30.0)
        self.declare_parameter('/GPSBoostMargin', 0.1745)

        self.following_direction = self.get_parameter('/FollowingDirection').value
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

        self.MAX_CHANGE = 5


    def __del__(self):
        pass

    def signal_catch(self, signum):
        pass

    def wheel_callback(self, msg):
        left_speed = self.curr_left_speed
        right_speed = self.curr_right_speed

        stop_override = False
        message_valid = True

        self.get_logger().info(f"received msg {msg}")

        # put all this code in place
        if msg[:3] == WHEELS_OBJECT_AVOIDANCE:
            self.following_mode = FollowMode.eeObject
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO OBJECT AVOIDANCE")
        elif msg[:3] == WHEELS_LINE_FOLLOWING:
            self.following_mode = FollowMode.eeGps
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO LINE FOLLOWING")
        elif msg[:3] == WHEELS_GPS_NAV:
            self.following_mode = FollowMode.eeGps
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO GPS NAVIGATION")
        elif msg[:3] == WHEELS_TRANSITION:
            self.following_mode = FollowMode.eeTransition
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO TRANSITION STATE")
        elif msg[:3] == TRANSITION_CODE:
            cmds = msg[4:].split(',')
            if len(cmds) != 2:
                self.get_logger().warning("ERROR: MISFORMATTED MESSAGE")
            else:
                left_speed = int(float(cmds[0])*self.unitChange)
                right_speed = int(float(cmds[1])*self.unitChange)
        elif self.following_mode==FollowMode.eeLine and msg[:3]==LIN_SENDER:
            position = float(msg[4:])
            if position >= STOP_CODE:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                position_error = self.target_line_dist - position
                # C++ does this stuff
                # // Calculate the differential and apply it
                # to the default speed double
                # delta = this->pidCtrLine->control(positionError);
                # delta = delta * (double)(this->followingPol);
                # leftSpeed = this->defaultSpeed + delta;
                # rightSpeed = this->defaultSpeed - delta;
                #
                # // Check if we should in the acceptable zone for picking up speed.i
                if abs(position_error) <= self.line_boost_margin:
                    self.boost_count += 1
                else:
                    self.boost_count = 0
        elif self.following_mode == FollowMode.eeObject and msg[:3]==OBJECT_SENDER:
            position = float(msg[4:])
            if position >= STOP_CODE:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                pass
                # c++ does this to calculate differntial and apply to default speed
                # double delta = this->pidCtrObj->control(this->targetObjDist - position);
                # delta = delta * (double)(this->followingPol);
                # leftSpeed = this->defaultSpeed + delta;
                # rightSpeed = this->defaultSpeed - delta;
        elif self.following_mode == FollowMode.eeGps and msg[:3] == GPS_SENDER:
            position = float(msg[4:])
            if position >= STOP_CODE:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                # more differential stuff
                # double delta = this->pidCtrGps->control(position);
                # GPS sends the error already
                # delta = delta * (double)(this->followingPol);
                # leftSpeed = this->defaultSpeed + delta;
                # rightSpeed = this->defaultSpeed - delta;
                if abs(position) <= self.gps_boost_margin:
                    self.boost_count += 1
                else:
                    self.boost_count = 0

        else:
            message_valid = False

        self.get_logger().info(f"Calculated left and right: {left_speed} and {right_speed}")
        if self.boost_count > self.boost_count_threshold and message_valid:
            left_speed += self.speed_boost
            right_speed += self.speed_boost

        # send speed command to wheels
        if message_valid and (left_speed != self.curr_left_speed or self.curr_right_speed != right_speed):
            if not stop_override:
                left_speed = self.curr_left_speed + self.MAX_CHANGE * int(left_speed > self.curr_left_speed + self.MAX_CHANGE)
                right_speed = self.curr_right_speed + self.MAX_CHANGE * int(right_speed > self.curr_right_speed + self.MAX_CHANGE)

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
