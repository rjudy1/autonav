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
from std_msgs.msg import String
from utils.utils import *

from .motor_driver import Wheels
from .pid_controller import PIDController


class WheelControl(Node):
    def __init__(self):
        super().__init__('wheels_controller')
        self.wheel_sub = self.create_subscription(String, "wheel_distance", self.wheel_callback, 10)
        self.state_sub = self.create_subscription(String, "state_topic", self.state_callback, 10)
        self.unitChange = 1  # assuming passed in meters, need mm

        # start in a stopped state
        self.curr_right_speed = 0
        self.curr_left_speed = 0
        self.boost_count = 0

        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/LineDist', 0.175)
        self.declare_parameter('/SideObjectDist', 0.6)
        self.declare_parameter('/DefaultSpeed', 15.0)
        self.declare_parameter('/BoostIncrease', 1)
        self.declare_parameter('/BoostCountThreshold', 20)
        self.declare_parameter('/LineBoostMargin', 30.0)
        self.declare_parameter('/GPSBoostMargin', 0.1745)
        self.declare_parameter('/Port', '/dev/ttyUSB0')

        self.following_direction = self.get_parameter('/FollowingDirection').value
        self.target_line_dist = self.get_parameter('/LineDist').value*1000
        self.target_obj_dist = self.get_parameter('/SideObjectDist').value
        self.default_speed = self.get_parameter('/DefaultSpeed').value
        self.speed_boost = self.get_parameter('/BoostIncrease').value
        self.boost_count_threshold = self.get_parameter('/BoostCountThreshold').value
        self.line_boost_margin = self.get_parameter('/LineBoostMargin').value
        self.gps_boost_margin = self.get_parameter('/GPSBoostMargin').value

        self.pid_line = PIDController(-0.03, 0.0, 0.0, 15, -15)  # for line following
        self.pid_obj = PIDController(0.025, 0.0, 1000.0, 15, -15)   # for object avoidance
        self.pid_gps = PIDController(2.5, 0.0, 0.0, 15, -15)   # for during gps navigation

        self.driver = Wheels(port=self.get_parameter('/Port').value)
        self.following_mode = FollowMode.eeLine
        self.STOP_LIMIT = 7777
        self.state = STATE.LINE_FOLLOWING

        self.MAX_CHANGE = 14
        self.get_logger().info("Launching motors")

    def __del__(self):
        self.driver.control_wheels(0,0)

    def state_callback(self, new_state):
        self.get_logger().info("New State Received: {}".format(new_state.data))
        self.state = int(new_state.data)

        # effects internal FollowMode
        if self.state == STATE.GPS_TO_OBJECT or self.state ==STATE.OBJECT_AVOIDANCE_FROM_GPS or\
            self.state == STATE.LINE_TO_OBJECT or self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:
            self.following_mode = FollowMode.eeObject
        elif self.state == STATE.LINE_FOLLOWING:
            self.following_mode = FollowMode.eeLine
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO LINE FOLLOWING")
        elif self.state == STATE.GPS_NAVIGATION:
            self.following_mode = FollowMode.eeGps
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO GPS NAVIGATION")
        elif self.state == STATE.OBJECT_TO_LINE or self.state == STATE.FIND_LINE or
            self.state = STATE.LINE_ORIENT:
            self.following_mode = FollowMode.eeTransition
            self.boost_count = 0
            self.get_logger().info("SWITCHED TO TRANSITION STATE")


    def wheel_callback(self, msg):
        self.get_logger().info(f"state: {self.following_mode}, {msg}")
        msg = msg.data
        left_speed = self.curr_left_speed
        right_speed = self.curr_right_speed

        stop_override = False
        message_valid = True

        if msg[:3] == CODE.TRANSITION_CODE:
            cmds = msg[4:].split(',')
            if len(cmds) != 2:
                self.get_logger().warning("ERROR: MISFORMATTED MESSAGE")
            else:
                left_speed = int(float(cmds[0])*self.unitChange)
                right_speed = int(float(cmds[1])*self.unitChange)
        elif self.following_mode==FollowMode.eeLine and msg[:3]==CODE.LIN_SENDER:
            self.get_logger().info(f"IN FOLLOW MODE: msg = {msg}")
            position = float(msg[4:])
            if position >= self.STOP_LIMIT:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                position_error = self.target_line_dist - position
                # self.get_logger().warning(f"POSITION ERROR {position_error}")
                # Calculate the differential and apply it
                # to the default speed double
                delta = self.pid_line.control(position_error)
                delta = delta / 2  if self.following_direction==DIRECTION.RIGHT else -1 * delta / 2
                self.get_logger().warning(f"compensating by {delta}")
                left_speed = self.default_speed + delta
                right_speed = self.default_speed - delta
                self.get_logger().info(f"left: {left_speed}, right: {right_speed}")
                #
                # // Check if we should in the acceptable zone for picking up speed.i
                if abs(position_error) <= self.line_boost_margin:
                    self.boost_count += 1
                else:
                    self.boost_count = 0
        elif self.following_mode == FollowMode.eeObject and msg[:3]==CODE.OBJECT_SENDER:
            self.get_logger().info(f"sending motor commands {msg}")
            position = float(msg[4:])
            if position >= self.STOP_LIMIT:
                left_speed = 0
                right_speed = 0
                stop_override = True
                self.boost_count = 0
            else:
                pass
                # c++ does this to calculate differntial and apply to default speed
                delta = self.pid_obj.control(self.target_obj_dist - position)
                delta = delta * self.following_direction
                left_speed = self.curr_left_speed - delta
                right_speed = self.curr_right_speed + delta
        elif self.following_mode == FollowMode.eeGps and msg[:3] == CODE.GPS_SENDER:
            position = float(msg[4:])
            # if position >= self.STOP_LIMIT:
            #     left_speed = 0
            #     right_speed = 0
            #     stop_override = True
            #     self.boost_count = 0
            # else:
            # more differential stuff
            delta = self.pid_gps.control(position)
            # GPS sends the error already
            delta = delta * self.following_direction
            left_speed = self.curr_left_speed - delta
            right_speed = self.curr_right_speed + delta
            if abs(position) <= self.gps_boost_margin:
                self.boost_count += 1
            else:
                self.boost_count = 0

        else:
            self.get_logger().info(f"INVALID MESSAGE{self.following_direction} : {msg}\n\n")
            message_valid = False

        # self.get_logger().info(f"Calculated left and right speeds: {left_speed} and {right_speed}")
        if self.boost_count > self.boost_count_threshold and message_valid:
            left_speed += self.speed_boost
            right_speed += self.speed_boost

        # send speed command to wheels
        # self.get_logger().info(f"left: {left_speed}. curleft: {self.curr_left_speed}; right: {right_speed}. curRight: {self.curr_right_speed}")
        if message_valid and (left_speed != self.curr_left_speed or self.curr_right_speed != right_speed):
            self.get_logger().info(f"ready to send {left_speed} and {right_speed} and stop is {stop_override}")
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
            self.get_logger().info(f"{self.driver.control_wheels(left_speed, right_speed)}")

        self.curr_left_speed = left_speed
        self.curr_right_speed = right_speed


def main(args=None):
    rclpy.init(args=args)
    wheels = WheelControl()

    try:
        rclpy.spin(wheels)
    except KeyboardInterrupt:
        # Destroy the node explicitly (optional)
        wheels.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
