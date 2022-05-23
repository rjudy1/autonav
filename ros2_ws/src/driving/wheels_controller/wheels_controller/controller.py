################################
# AutoNav 2022 Competition Robot
# Package: wheel_controller
# File: controller.py
# Purpose: control wheels
# Date Modified: 21 May 2022
################################

# !/usr/bin/env python

from custom_msgs.msg import SpeedCmd
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
import time
from utils.utils import *


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
        right_cmd = self.convert_to_hex_cmd(right_speed) | RIGHT_WHEEL
        self.serialPort.write(right_cmd.to_bytes(1, 'big'))

        # control left wheel
        left_cmd = self.convert_to_hex_cmd(left_speed)
        self.serialPort.write(left_cmd.to_bytes(1, 'big'))

        self.serialPort.flush()

    def convert_to_hex_cmd(self, speed):
        # get magnitude of speed and direction
        cmd = abs(speed) & 0xFF
        if speed < 0:
            cmd = (cmd | CCW)
        return cmd


class PIDController:
    def __init__(self, kp, ki, kd, max, min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_val = max
        self.min_val = min
        self.last_time = time.time_ns()
        self.prev_error = 0.0
        self.prev_integral_errors = [0.0 for i in range(10)]

    def control(self, curr_error):
        curr_time = time.time_ns()
        iteration_time = (curr_time - self.last_time) * 1000 # microseconds

        prev_int_error = 0.0
        for error in self.prev_integral_errors:
            prev_int_error += error

        integral_term = prev_int_error + curr_error*iteration_time
        derivative_term = (curr_error - self.prev_error)/iteration_time

        output = self.kp*curr_error + self.ki*integral_term + self.kd*derivative_term

        self.prev_error = curr_error

        self.prev_integral_errors.pop(0)
        self.prev_integral_errors.append(curr_error)
        
        self.last_time = curr_time

        if output > self.max_val:
            output = self.max_val
        elif output < self.min_val:
            output = self.min_val

        return output


class WheelControl(Node):
    def __init__(self):
        super().__init__('wheels_controller')
        self.wheel_sub = self.create_subscription(String, "wheel_distance", self.wheel_callback, 10)
        self.speed_cmd_pub = self.create_publisher(SpeedCmd, 'speed_cmds', 10)
        self.unitChange = 1000  # assuming passed in meters, need mm

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
        self.declare_parameter('/Port', '/dev/ttyUSB0')

        self.following_direction = self.get_parameter('/FollowingDirection').value
        self.target_line_dist = self.get_parameter('/LineDist').value
        self.target_obj_dist = self.get_parameter('/SideObjectDist').value
        self.default_speed = self.get_parameter('/DefaultSpeed').value
        self.speed_boost = self.get_parameter('/BoostIncrease').value
        self.boost_count_threshold = self.get_parameter('/BoostCountThreshold').value
        self.line_boost_margin = self.get_parameter('/LineBoostMargin').value
        self.gps_boost_margin = self.get_parameter('/GPSBoostMargin').value

        self.pid_line = PIDController(-0.05, 0.0, 0.0, 15, -15)  # for lie following
        self.pid_obj = PIDController(0.025, 0.0, 1000.0, 15, -15)   # for object avoidance
        self.pid_gps = PIDController(2.5, 0.0, 0.0, 15, -15)   # for during gps navigation

        self.driver = Wheels(port=self.get_parameter('/Port').value)
        self.following_mode = FollowMode.eeLine

        self.MAX_CHANGE = 5

    def __del__(self):
        self.driver.control_wheels(0,0)

    def wheel_callback(self, msg):
        msg = msg.data
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
            self.get_logger().warning(f"HELLOOOOO: {cmds}")
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
                # Calculate the differential and apply it
                # to the default speed double
                delta = self.pid_line.control(position_error)
                delta = delta * self.following_direction
                left_speed = self.default_speed + delta
                right_speed = self.default_speed - delta
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
                delta = self.pid_obj.control(self.target_obj_dist - position)
                delta = delta * self.following_direction
                left_speed = self.default_speed + delta
                right_speed = self.default_speed - delta;
        elif self.following_mode == FollowMode.eeGps and msg[:3] == GPS_SENDER:
            position = float(msg[4:])
            if position >= STOP_CODE:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                # more differential stuff
                delta = self.pid_gps.control(position)
                # GPS sends the error already
                delta = delta * self.following_direction
                left_speed = self.default_speed + delta
                right_speed = self.default_speed - delta
                if abs(position) <= self.gps_boost_margin:
                    self.boost_count += 1
                else:
                    self.boost_count = 0

        else:
            message_valid = False

        self.get_logger().info(f"Calculated left and right speeds: {left_speed} and {right_speed}")
        if self.boost_count > self.boost_count_threshold and message_valid:
            left_speed += self.speed_boost
            right_speed += self.speed_boost

        # send speed command to wheels
        if message_valid and (left_speed != self.curr_left_speed or self.curr_right_speed != right_speed):
            if not stop_override:
                if left_speed > self.curr_left_speed + self.MAX_CHANGE:
                    left_speed = self.curr_left_speed + self.MAX_CHANGE
                elif left_speed < self.curr_left_speed - self.MAX_CHANGE:
                    left_speed = self.curr_left_speed - self.MAX_CHANGE
                if right_speed > self.curr_right_speed + self.MAX_CHANGE:
                    right_speed = self.curr_right_speed + self.MAX_CHANGE
                elif right_speed < self.curr_right_speed - self.MAX_CHANGE:
                    right_speed = self.curr_right_speed - self.MAX_CHANGE
            # self.get_logger().info(f"SETTING WHEEL SPEED TO {left_speed} and {right_speed}")
            self.driver.control_wheels(left_speed, right_speed)

        self.curr_left_speed = left_speed
        self.curr_right_speed = right_speed


def main(args=None):
    rclpy.init(args=args)
    wheels = WheelControl()

    try:
        rclpy.spin(wheels)
    except Exception as e:
        # Destroy the node explicitly (optional)
        wheels.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
