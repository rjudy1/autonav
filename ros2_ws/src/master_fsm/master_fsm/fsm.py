#!/usr/bin/env python3

################################
# AutoNav 2022 Competition Robot
# Package: master_fsm
# File: fsm.py
# Purpose: robot big brain based on 2021 brain
# Date Modified: 21 May 2022
################################
import sys

sys.path.insert(1, '/home/autonav/autonav')

import rclpy
from rclpy.node import Node
import threading
import time
from utils import *

from std_msgs.msg import String


class MainRobot(Node):
    def __init__(self):
        super().__init__("main_robot")

        self.declare_parameter('/DefaultSpeed', 15)
        self.declare_parameter('/FollowingDirection', DIRECTION.RIGHT)
        self.declare_parameter('/TimerRate', .05)
        self.declare_parameter('/StartState', STATE.LINE_FOLLOWING)

        # Make a lock so the callbacks don't create race conditions
        self.lock = threading.Lock()

        # Publish to the wheels, lights, state changes
        self.wheel_pub = self.create_publisher(String, "wheel_distance", 10)
        self.lights_pub = self.create_publisher(String, "light_state", 10)
        self.state_pub = self.create_publisher(String, "state_topic", 100)

        # Subscribe to new event notifications from the lights node, gps, lidar
        self.line_sub = self.create_subscription(String, "line_events", self.line_callback, 10)
        self.gps_sub = self.create_subscription(String, "gps_events", self.gps_callback, 10)
        self.depth_sub = self.create_subscription(String, "/mod_lidar", self.lidar_callback, 10)

        # already declared messages to save a couple lines
        self.state_msg = String()
        self.wheel_msg = String()

        self.state = self.get_parameter("/StartState").value
        self.state_msg.data = str(self.state)
        self.state_pub.publish(self.state_msg)
        self.wheel_msg.data = WHEELS_TRANSITION if self.state!=STATE.LINE_FOLLOWING else WHEELS_LINE_FOLLOWING
        self.wheel_pub.publish(self.wheel_msg)
        self.get_logger().info("Initializing Main Robot Controller...")

        self.transition_set = {STATE.LINE_TO_OBJECT, STATE.GPS_TO_OBJECT}

        self.obj_seen = False
        self.found_line = False
        self.aligned = False
        self.waypoint_found = False
        self.waypoint_straight = False
        self.path_clear = False
        self.follow_dir = self.get_parameter('/FollowingDirection')

        # Make a timer object for calling the change state periodically
        self.timer = self.create_timer(self.get_parameter('/TimerRate').value, self.timer_callback)

    # Beginning of State Machine

    # Beginning of Major States

    # Line Following State
    def line_following_state(self):
        self.get_logger().info("Line Following State")
        if self.waypoint_found:  # reached gps waypoint - switch to gps navigation
            self.waypoint_found = False
            self.state = STATE.GPS_NAVIGATION
            self.state_msg.data = str(STATE.GPS_NAVIGATION)
            self.state_pub.publish(self.state_msg)
            self.safe_publish(self.wheel_pub, WHEELS_GPS_NAV)
            self.gps_navigation_state()
        elif self.obj_seen:  # object sighted - switch to obstacle avoidance
            # We check for an object second because if we have already hit the
            # GPS waypoint we want the robot to record that first.
            self.obj_seen = False
            self.state = STATE.LINE_TO_OBJECT
            self.state_msg.data = str(STATE.LINE_TO_OBJECT)
            self.state_pub.publish(self.state_msg)

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)

            self.line_to_object_state()  # enter the transition state

    # Object Avoidance From Line Following State
    def object_avoidance_from_line_state(self):
        self.get_logger().info("Object Avoidance From Line Following State")
        # Check for another object in front of the robot
        if self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = str(STATE.LINE_TO_OBJECT)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_TO_OBJECT

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.line_to_object_state()  # enter the transition state
        elif self.found_line:  # Otherwise see if we found the line.
            self.found_line = False
            self.state_msg.data = str(STATE.OBJECT_TO_LINE)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_TO_LINE

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.object_to_line_state()  # enter the transition state

    # Object Avoidance From GPS Navigation State
    def object_avoidance_from_gps_state(self):
        self.get_loggger().info("Object Avoidance From GPS State")

        # Check for another object in front of the robot
        if self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = str(STATE.GPS_TO_OBJECT)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_TO_OBJECT

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.gps_to_object_state()  # enter the transition state
        elif self.waypoint_straight:  # Otherwise see if have a clear path to the waypoint
            self.waypoint_straight = False
            self.state_msg.data = str(STATE.GPS_NAVIGATION)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_NAVIGATION

            self.wheel_msg.data = WHEELS_GPS_NAV
            self.wheel_pub.publish(self.wheel_msg)
            self.gps_navigation_state()  # enter the gps navigation state

    # GPS Navigation State
    def gps_navigation_state(self):
        self.get_logger().info("GPS Navigation State")

        # First look for a potential obstacle
        if self.obj_seen:
            self.obj_seen = False
            self.state_msg.data = str(STATE.GPS_TO_OBJECT)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.GPS_TO_OBJECT

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.gps_to_object_state()
        # After looking for an obstacle, see if we have arrived
        elif self.waypoint_found:
            self.waypoint_found = False
            self.state_msg.data = str(STATE.FIND_LINE)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.FIND_LINE

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.find_line_state()

    # End of Major States

    # Beginning of Transition States
    # In these states the main controller (this class) is
    # the one controlling the wheels

    # Line Following to Object Avoidance Transition State
    def line_to_object_state(self):
        self.get_logger().info("Line to Object Transition State")
        # Just keep turning until the object is not in front of us
        self.wheel_msg = TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * TURN_SPEED) + "," + str(
            TURN_SPEED + self.follow_dir * TURN_SPEED)
        self.wheel_pub.publish(self.wheel_msg)

        if self.path_clear:
            self.path_clear = False
            self.state_msg.data = str(STATE.OBJECT_AVOIDANCE_FROM_LINE)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_AVOIDANCE_FROM_LINE

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.object_avoidance_from_line_state()

    # Object Avoidance to Line Following Transition State
    def object_to_line_state(self):
        self.get_logger().info("Object to Line Transition State")

        # Gradual Turn
        self.wheel_msg.data = TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * SLIGHT_TURN) + "," + str(
            TURN_SPEED + self.follow_dir * SLIGHT_TURN)
        self.wheel_pub.publish(self.wheel_msg)

        # Soldier Turn
        # self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(TURN_SPEED - self.follow_dir*TURN_SPEED) + "," + str(TURN_SPEED + self.follow_dir*TURN_SPEED))

        # Just keep turning until we are parallel with the line
        if self.aligned:
            self.aligned = False
            self.state_msg.data = str(STATE.LINE_FOLLOWING)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_FOLLOWING

            self.wheel_msg.data = WHEELS_LINE_FOLLOWING
            self.wheel_pub.publish(self.wheel_msg)
            self.line_following_state()

    # GPS Navigation to Object Avoidance Transition State
    def gps_to_object_state(self):
        self.get_logger().info("GPS to Object Transition State")

        # Just keep turning until the object is not in front of us
        self.wheel_msg.data = TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * TURN_SPEED) + "," + str(
            TURN_SPEED + self.follow_dir * TURN_SPEED)
        self.wheel_pub.publish(self.wheel_msg)
        if self.path_clear:
            self.path_clear = False
            self.state_msg.data = str(STATE.OBJECT_AVOIDANCE_FROM_GPS)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.OBJECT_AVOIDANCE_FROM_GPS

            self.wheel_msg.data = WHEELS_OBJECT_AVOIDANCE
            self.wheel_pub.publish(self.wheel_msg)
            self.object_avoidance_from_gps_state()

    # Transition State to find the line after GPS Navigation
    def find_line_state(self):
        self.get_logger().info("Find Line Transition State")
        # Just keep going until we find the line
        self.wheel_msg.data = TRANSITION_CODE + str(0) + "," + str(0)
        self.wheel_pub.publish(self.wheel_msg)

        if self.found_line:
            self.found_line = False
            self.state_msg.data = str(STATE.LINE_ORIENT)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_ORIENT

            self.wheel_msg.data = WHEELS_TRANSITION
            self.wheel_pub.publish(self.wheel_msg)
            self.line_orientation_state()  # Go to the next transition state

    # Transition State to Orient to the line direction
    def line_orientation_state(self):
        self.get_logger().info("Line Orientation Transition State")

        # Just keep turning until we are parrallel with the line
        self.wheel_msg.data = TRANSITION_CODE + str(0) + "," + str(0)
        self.wheel_pub.publish(self.wheel_msg)

        if self.aligned:
            self.aligned = False
            self.state_msg.data = str(STATE.LINE_FOLLOWING)
            self.state_pub.publish(self.state_msg)
            self.state = STATE.LINE_FOLLOWING

            self.safe_publish(self.wheel_pub, WHEELS_LINE_FOLLOWING)
            self.line_following_state()

    # End of Transition States

    # This function is essentially a big state machine handling transitions
    # between a number of different states in the system.
    def change_state(self):
        self.get_logger().info(f"state {self.state}")
        if self.state == STATE.LINE_FOLLOWING:  self.line_following_state()
        elif self.state == STATE.OBJECT_AVOIDANCE_FROM_LINE:  self.object_avoidance_from_line_state()
        elif self.state == STATE.OBJECT_AVOIDANCE_FROM_GPS:  self.object_avoidance_from_gps_state()
        elif self.state == STATE.GPS_NAVIGATION:  self.gps_navigation_state()
        elif self.state == STATE.LINE_TO_OBJECT:  self.line_to_object_state()
        elif self.state == STATE.OBJECT_TO_LINE:  self.object_to_line_state()
        elif self.state == STATE.GPS_TO_OBJECT:  self.gps_to_object_state()
        elif self.state == STATE.FIND_LINE:  self.find_line_state()
        elif self.state == STATE.LINE_ORIENT:  self.line_orientation_state()
        else: self.get_logger().info("Error: Invalid State")

    # End of State Machine

    # Beginning of Callback Methods

    # Callback for information coming from the line following node
    def line_callback(self, line_event):
        self.get_logger().info("Message from Line Following Node")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            if line_event.data == FOUND_LINE:
                self.get_logger().warning("FOUND LINE!!")
                self.found_line = True
            elif line_event.data == ALIGNED:
                self.aligned = True
            else:
                self.get_logger().info("UNKNOWN MESSAGE")
        finally:
            # Release the lock
            self.lock.release()

    # Callback for information coming from the GPS node
    def gps_callback(self, gps_event):
        self.get_logger().info("Message from GPS Node")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            if gps_event.data == WAYPOINT_FOUND:
                self.waypoint_found = True
            elif gps_event.data == WAYPOINT_STRAIGHT:
                self.waypoint_straight = True
            else:
                self.get_logger().warning("Unknown Message")
        finally:
            # Release the lock
            self.lock.release()

    # Callback for information from the depth camera
    def lidar_callback(self, lidar_event):
        self.get_logger().info("Message from Depth Camera")

        # Get the lock before proceeding
        self.lock.acquire()
        try:
            if lidar_event.data == OBJECT_SEEN:
                self.obj_seen = True
            elif (lidar_event.data == PATH_CLEAR) and (self.STATE in self.transition_set):
                self.path_clear = True
            else:
                self.get_logger().info(f"Unknown Message {lidar_event}")
        finally:
            # Release the lock
            self.lock.release()

    # Callback for the timer
    def timer_callback(self):
        # self.get_logger().info("Timer Callback")

        # Get the lock before proceeding
        self.lock.acquire()
        try:
            self.change_state()
        finally:
            # Release the lock
            self.lock.release()


def main():
    rclpy.init()
    main = MainRobot()
    time.sleep(3)
    # Get the lock so the Timer doesn't interfere
    main.lock.acquire()

    try:
        # Start the Robot
        main.change_state()
        main.lock.release()
        rclpy.spin(main)
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
