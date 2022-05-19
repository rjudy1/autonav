#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Main Class                                    #
# ********************************************* #

import sys

sys.path.insert(1, '/home/autonav/autonav/ros1_ws/src/autonav_utils/')

import cv2
import rospy
import threading
import time
from autonav_node import AutoNavNode
from enum import Enum
from std_msgs.msg import String

# Messages that change the wheel controller's state
WHEELS_TRANSITION = "STR"
WHEELS_OBJECT_AVOIDANCE = "SOA"
WHEELS_LINE_FOLLOWING = "SLF"
WHEELS_GPS_NAV = "SGN"
STOP_CODE = "STO"
TRANSITION_CODE = "TRA,"
TURN_SPEED = 14
SLIGHT_TURN = 10

# Messages that indicate a change of state is needed
PATH_CLEAR = "PATH_CLEAR"
OBJECT_SEEN = "OBJECT_SEEN"
WAYPOINT_FOUND = "WAYPOINT_FOUND"
WAYPOINT_STRAIGHT = "WAYPOINT_STRAIGHT"
FOUND_LINE = "FOUND_LINE"
ALIGNED = "ALIGNED"


# States for the FSM
class State(Enum):
    Line_Following = 1
    Object_Avoidance_From_Line = 2
    Object_Avoidance_From_GPS = 3
    GPS_Navigation = 4
    Line_To_Object = 5
    Object_To_Line = 6
    GPS_To_Object = 7
    Find_Line = 8
    Line_Orientation = 9


class MainRobot(AutoNavNode):

    def __init__(self):
        AutoNavNode.__init__(self, "main_robot")

        self.read_param("/DefaultSpeed", "default_speed", 15)
        self.read_param("/FollowingDirection", "follow_dir", 1)
        self.read_param("/TimerRate", "timer_rate", 0.05)
        self.read_param("/StartState", "start_state", 1)

        # Make a lock so the callbacks don't create race conditions
        self.lock = threading.Lock()

        # Publish to the wheels
        self.wheel_pub = rospy.Publisher("wheel_distance", String, queue_size=10)
        # Publish to the lights
        self.lights_pub = rospy.Publisher("light_state", String, queue_size=10)
        # Publish the new state on state changes (to everyone)
        self.state_pub = rospy.Publisher("state_topic", String, queue_size=100)
        # Subscribe to new event notifications from the lights node
        self.line_sub = rospy.Subscriber("line_events", String, self.line_callback)
        # Subscribe to new event notifications from the gps node
        self.gps_sub = rospy.Subscriber("gps_events", String, self.gps_callback)
        # Subscribe to new event notifications from the depth camera
        self.depth_sub = rospy.Subscriber("/mod_lidar", String, self.depth_callback)

        self.state = State.Line_Following
        rospy.loginfo("Initializing Main Robot Controller...")

        self.transition_set = {State.Line_To_Object, State.GPS_To_Object}

        self.obj_seen = False
        self.found_line = False
        self.aligned = False
        self.waypoint_found = False
        self.waypoint_straight = False
        self.path_clear = False

        # Make a timer object for calling the change state periodically
        self.timer = rospy.Timer(rospy.Duration(self.timer_rate), self.timer_callback)

    # Beginning of State Machine

    # Beginning of Major States

    # Line Following State
    def line_following_state(self):
        rospy.loginfo("Line Following State")
        self.safe_publish(self.lights_pub, "R_On")
        self.safe_publish(self.lights_pub, "Y_Off")
        self.safe_publish(self.lights_pub, "G_Off")
        if self.waypoint_found:  # reached gps waypoint - switch to gps navigation
            self.waypoint_found = False
            self.safe_publish(self.state_pub, self)
            self.state = State.GPS_Navigation
            self.safe_publish(self.wheel_pub, WHEELS_GPS_NAV)
            self.gps_navigation_state()
        elif self.obj_seen:  # object sighted - switch to obstacle avoidance
            # We check for an object second because if we have already hit the 
            # GPS waypoint we want the robot to record that first.
            self.obj_seen = False
            self.safe_publish(self.state_pub, self.LINE_TO_OBJECT)
            self.state = State.Line_To_Object
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.line_to_object_state()  # enter the transition state

    # Object Avoidance From Line Following State
    def object_avoidance_from_line_state(self):
        rospy.loginfo("Object Avoidance From Line Following State")
        self.safe_publish(self.lights_pub, "R_On")
        self.safe_publish(self.lights_pub, "Y_On")
        self.safe_publish(self.lights_pub, "G_Off")
        # Check for another object in front of the robot
        if self.obj_seen:
            self.obj_seen = False
            self.safe_publish(self.state_pub, self.LINE_TO_OBJECT)
            self.state = State.Line_To_Object
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.line_to_object_state()  # enter the transition state
        elif self.found_line:  # Otherwise see if we found the line.
            self.found_line = False
            self.safe_publish(self.state_pub, self.OBJECT_TO_LINE)
            self.state = State.Object_To_Line
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.object_to_line_state()  # enter the transition state

    # Object Avoidance From GPS Navigation State
    def object_avoidance_from_gps_state(self):
        rospy.loginfo("Object Avoidance From GPS State")
        self.safe_publish(self.lights_pub, "R_Off")
        self.safe_publish(self.lights_pub, "Y_On")
        self.safe_publish(self.lights_pub, "G_On")

        # Check for another object in front of the robot
        if self.obj_seen:
            self.obj_seen = False
            self.safe_publish(self.state_pub, self.GPS_TO_OBJECT)
            self.state = State.GPS_To_Object
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.gps_to_object_state()  # enter the transition state
        elif self.waypoint_straight:  # Otherwise see if have a clear path to the waypoint
            self.waypoint_straight = False
            self.safe_publish(self.state_pub, self.GPS_NAVIGATION)
            self.state = State.GPS_Navigation
            self.safe_publish(self.wheel_pub, WHEELS_GPS_NAV)
            self.gps_navigation_state()  # enter the gps navigation state

    # GPS Navigation State
    def gps_navigation_state(self):
        rospy.loginfo("GPS Navigation State")
        self.safe_publish(self.lights_pub, "R_Off")
        self.safe_publish(self.lights_pub, "Y_Off")
        self.safe_publish(self.lights_pub, "G_On")

        # First look for a potential obstacle
        if self.obj_seen:
            self.obj_seen = False
            self.safe_publish(self.state_pub, self.GPS_TO_OBJECT)
            self.state = State.GPS_To_Object
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.gps_to_object_state()
        # After looking for an obstacle, see if we have arrived
        elif self.waypoint_found:
            self.waypoint_found = False
            self.safe_publish(self.state_pub, self.FIND_LINE)
            self.state = State.Find_Line
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.find_line_state()

    # End of Major States

    # Beginning of Transition States
    # In these states this main controller (this class) is 
    # the one controlling the wheels

    # Line Following to Object Avoidance Transition State
    def line_to_object_state(self):
        rospy.loginfo("Line to Object Transition State")
        self.safe_publish(self.lights_pub, "R_On")
        self.safe_publish(self.lights_pub, "Y_Off")
        self.safe_publish(self.lights_pub, "G_On")

        # Just keep turning until the object is not in front of us
        self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * TURN_SPEED) + "," + str(
            TURN_SPEED + self.follow_dir * TURN_SPEED))

        if self.path_clear:
            self.path_clear = False
            self.safe_publish(self.state_pub, self.OBJECT_AVOIDANCE_FROM_LINE)
            self.state = State.Object_Avoidance_From_Line
            self.safe_publish(self.wheel_pub, WHEELS_OBJECT_AVOIDANCE)
            self.object_avoidance_from_line_state()

    # Object Avoidance to Line Following Transition State
    def object_to_line_state(self):
        rospy.loginfo("Object to Line Transition State")
        self.safe_publish(self.lights_pub, "R_On")
        self.safe_publish(self.lights_pub, "Y_On")
        self.safe_publish(self.lights_pub, "G_On")

        # Gradual Turn
        self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * SLIGHT_TURN) + "," + str(
            TURN_SPEED + self.follow_dir * SLIGHT_TURN))

        # Soldier Turn
        # self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(TURN_SPEED - self.follow_dir*TURN_SPEED) + "," + str(TURN_SPEED + self.follow_dir*TURN_SPEED))

        # Just keep turning until we are parrallel with the line
        if self.aligned:
            self.aligned = False
            self.safe_publish(self.state_pub, self.LINE_FOLLOWING)
            self.state = State.Line_Following
            self.safe_publish(self.wheel_pub, WHEELS_LINE_FOLLOWING)
            self.line_following_state()

    # GPS Navigation to Object Avoidance Transition State
    def gps_to_object_state(self):
        rospy.loginfo("GPS to Object Transition State")
        self.safe_publish(self.lights_pub, "R_Off")
        self.safe_publish(self.lights_pub, "Y_On")
        self.safe_publish(self.lights_pub, "G_Off")

        # Just keep turning until the object is not in front of us
        self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(TURN_SPEED - self.follow_dir * TURN_SPEED) + "," + str(
            TURN_SPEED + self.follow_dir * TURN_SPEED))
        if self.path_clear:
            self.path_clear = False
            self.safe_publish(self.state_pub, self.OBJECT_AVOIDANCE_FROM_GPS)
            self.state = State.Object_Avoidance_From_GPS
            self.safe_publish(self.wheel_pub, WHEELS_OBJECT_AVOIDANCE)
            self.object_avoidance_from_gps_state()

    # Transition State to find the line after GPS Navigation
    def find_line_state(self):
        rospy.loginfo("Find Line Transition State")
        self.safe_publish(self.lights_pub, "R_Off")
        self.safe_publish(self.lights_pub, "Y_Off")
        self.safe_publish(self.lights_pub, "G_Off")

        # Just keep going until we find the line
        self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(0) + "," + str(0))

        if self.found_line:
            self.found_line = False
            self.safe_publish(self.state_pub, self.LINE_ORIENT)
            self.state = State.Line_Orientation
            self.safe_publish(self.wheel_pub, WHEELS_TRANSITION)
            self.line_orientation_state()  # Go to the next transition state

    # Transition State to Orient to the line direction
    def line_orientation_state(self):
        rospy.loginfo("Line Orientation Transition State")
        self.safe_publish(self.lights_pub, "R_On")
        self.safe_publish(self.lights_pub, "Y_On")
        self.safe_publish(self.lights_pub, "G_On")

        # Just keep turning until we are parrallel with the line
        self.safe_publish(self.wheel_pub, TRANSITION_CODE + str(0) + "," + str(0))

        if self.aligned:
            self.aligned = False
            self.safe_publish(self.state_pub, self.LINE_FOLLOWING)
            self.state = State.Line_Following
            self.safe_publish(self.wheel_pub, WHEELS_LINE_FOLLOWING)
            self.line_following_state()

    # End of Transition States

    # This function is essentially a big state machine handling transitions
    # between a number of different states in the system.
    def change_state(self):
        if self.state == State.Line_Following: self.line_following_state()
        elif self.state == State.Object_Avoidance_From_Line: self.object_avoidance_from_line_state()
        elif self.state == State.Object_Avoidance_From_GPS: self.object_avoidance_from_gps_state()
        elif self.state == State.GPS_Navigation: self.gps_navigation_state()
        elif self.state == State.Line_To_Object: self.line_to_object_state()
        elif self.state == State.Object_To_Line: self.object_to_line_state()
        elif self.state == State.GPS_To_Object: self.gps_to_object_state()
        elif self.state == State.Find_Line: self.find_line_state()
        elif self.state == State.Line_Orientation: self.line_orientation_state()
        else: rospy.loginfo("Error: Invalid State")

    # End of State Machine

    # Beginning of Callback Methods

    # Callback for information coming from the line following node
    def line_callback(self, line_event):
        rospy.loginfo("Message from Line Following Node")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            if line_event.data == FOUND_LINE:
                rospy.logwarn("FOUNDLIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINNNNNEE")
                self.found_line = True
            elif line_event.data == ALIGNED:
                self.aligned = True
            else:
                rospy.logwarn("Unknown Message: {}".format(line_event.data))
        finally:
            # Release the lock
            self.lock.release()

    # Callback for information coming from the GPS node
    def gps_callback(self, gps_event):
        rospy.loginfo("Message from GPS Node")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            if gps_event.data == WAYPOINT_FOUND:
                self.waypoint_found = True
            elif gps_event.data == WAYPOINT_STRAIGHT:
                self.waypoint_straight = True
            else:
                rospy.logwarn("Unknown Message")
                rospy.logwarn(gps_event.data)
        finally:
            # Release the lock
            self.lock.release()


# this will have to be modified
    # Callback for information from the depth camera
    def depth_callback(self, depth_event):
        rospy.loginfo("Message from Depth Camera")

        # Get the lock before proceeding
        self.lock.acquire()
        try:
            if depth_event.data == OBJECT_SEEN:
                self.obj_seen = True
            elif (depth_event.data == PATH_CLEAR) and (self.state in self.transition_set):
                self.path_clear = True
            else:
                rospy.logwarn("Unknown Message")
                rospy.logwarn(depth_event.data)
        finally:
            # Release the lock
            self.lock.release()

    # Callback for the timer
    def timer_callback(self, timer):
        rospy.loginfo("Timer Callback")

        # Get the lock before proceeding
        self.lock.acquire()

        try:
            self.change_state()
        finally:
            # Release the lock
            self.lock.release()


def main(args):
    main = MainRobot()
    time.sleep(3)
    # Get the lock so the Timer doesn't interfere
    main.lock.acquire()

    # For use when testing:
    # To start the robot in a particular state, 
    # replace the False condition in the if statement
    # to check for the debug parameter for the statement
    # that chooses the state you want the robot to 
    # start in. If no state is chosen, the default state
    # will be selected.
    if main.start_state == State.Object_Avoidance_From_Line:  # Object Detection From Line Following State
        main.state = State.Object_Avoidance_From_Line
        main.safe_publish(main.state_pub, main.OBJECT_AVOIDANCE_FROM_LINE)
        #         main.state_pub.publish(main.OBJECT_AVOIDANCE_FROM_LINE)
        main.safe_publish(main.wheel_pub, WHEELS_OBJECT_AVOIDANCE)
    #         main.wheel_pub.publish(WHEELS_OBJECT_AVOIDANCE)
    elif main.start_state == State.Object_Avoidance_From_GPS:  # Object Detection From GPS Navigation State
        main.state = State.Object_Avoidance_From_GPS
        main.safe_publish(main.state_pub, main.OBJECT_AVOIDANCE_FROM_GPS)
        #         main.state_pub.publish(main.OBJECT_AVOIDANCE_FROM_GPS)
        main.safe_publish(main.wheel_pub, WHEELS_OBJECT_AVOIDANCE)
    #         main.wheel_pub.publish(WHEELS_OBJECT_AVOIDANCE)
    elif main.start_state == State.GPS_Navigation:  # GPS Navigation State
        main.state = State.GPS_Navigation
        main.safe_publish(main.state_pub, main.GPS_NAVIGATION)
        #         main.state_pub.publish(main.GPS_NAVIGATION)
        main.safe_publish(main.wheel_pub, WHEELS_GPS_NAV)
    #         main.wheel_pub.publish(WHEELS_GPS_NAV)
    # Transition States
    elif main.start_state == State.Line_To_Object:  # Line to Object Transition State
        main.state = State.Line_To_Object
        main.safe_publish(main.state_pub, main.LINE_TO_OBJECT)
        #         main.state_pub.publish(main.LINE_TO_OBJECT)
        main.safe_publish(main.wheel_pub, WHEELS_TRANSITION)
    #         main.wheel_pub.publish(WHEELS_TRANSITION)
    elif main.start_state == State.Object_To_Line:  # Object to Line Transition State
        main.state = State.Object_To_Line
        main.safe_publish(main.state_pub, main.OBJECT_TO_LINE)
        #         main.state_pub.publish(main.OBJECT_TO_LINE)
        main.safe_publish(main.wheel_pub, WHEELS_TRANSITION)
    #         main.wheel_pub.publish(WHEELS_TRANSITION)
    elif main.start_state == State.GPS_To_Object:  # GPS to Object Transition State
        main.state = State.GPS_To_Object
        main.safe_publish(main.state_pub, main.GPS_TO_OBJECT)
        #         main.state_pub.publish(main.GPS_TO_OBJECT)
        main.safe_publish(main.wheel_pub, WHEELS_TRANSITION)
    #         main.wheel_pub.publish(WHEELS_TRANSITION)
    elif main.start_state == State.Find_Line:  # Find Line Transition State
        main.state = State.Find_Line
        main.safe_publish(main.state_pub, main.FIND_LINE)
        #         main.state_pub.publish(main.FIND_LINE)
        main.safe_publish(main.wheel_pub, WHEELS_TRANSITION)
    #         main.wheel_pub.publish(WHEELS_TRANSITION)
    elif main.start_state == State.Line_Orientation:  # Change to Line Orientation State
        main.state = State.Line_Orientation
        main.safe_publish(main.state_pub, main.LINE_ORIENT)
        #         main.state_pub.publish(main.LINE_ORIENT)
        main.safe_publish(main.wheel_pub, WHEELS_TRANSITION)
    #         main.wheel_pub.publish(WHEELS_TRANSITION)
    else:  # Default State: Line Following
        main.state = State.Line_Following
        main.safe_publish(main.state_pub, main.LINE_FOLLOWING)
        #         main.state_pub.publish(main.LINE_FOLLOWING)
        main.safe_publish(main.wheel_pub, WHEELS_LINE_FOLLOWING)
    #         main.wheel_pub.publish(WHEELS_LINE_FOLLOWING)

    try:
        # Start the Robot
        main.change_state()
        main.lock.release()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
