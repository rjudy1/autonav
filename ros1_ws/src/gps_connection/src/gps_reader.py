#!/usr/bin/env python

# Joshua Kortje
# September 2020
# UART Connection Script

import serial
import time
import string
import pynmea2
import rospy
import cmath
import sys
sys.path.insert(1, '/home/autonav/autonav/ros1_ws/src/autonav_utils/')
from geopy import distance
import numpy as np
from threading import Thread
from std_msgs.msg import String
from autonav_node import AutoNavNode


class GPS(AutoNavNode):
    def __init__(self):
        AutoNavNode.__init__(self, "gps_node")

        self.WAYPOINT_STRAIGHT = "WAYPOINT_STRAIGHT"
        
        self.read_param("/WaypointLat1", "WP_LAT1", 0.0)
        self.read_param("/WaypointLon1", "WP_LON1", 0.0)
        self.read_param("/WaypointLat2", "WP_LAT2", 0.0)
        self.read_param("/WaypointLon2", "WP_LON2", 0.0)
        self.read_param("/WaypointLat3", "WP_LAT3", 0.0)
        self.read_param("/WaypointLon3", "WP_LON3", 0.0)
        self.read_param("/WaypointLat4", "WP_LAT4", 0.0)
        self.read_param("/WaypointLon4", "WP_LON4", 0.0)
        self.read_param("/ExitAngle", "exit_angle", 0.2)
        self.read_param("/GPSFollowGoal", "DISTANCE_GOAL", 1.0)
        self.read_param("/LineToGPSTrans", "TRANS_DIST", 5.0)


        self.target_loc = []
        self.target_loc.append(complex(self.WP_LAT1, self.WP_LON1))
        self.target_loc.append(complex(self.WP_LAT2, self.WP_LON2))
        self.target_loc.append(complex(self.WP_LAT3, self.WP_LON3))
        self.target_loc.append(complex(self.WP_LAT4, self.WP_LON4))

        self.waypoint_itr = 0

        # Publish new events that may change the overall state of the robot
        self.event_pub = rospy.Publisher("gps_events", String, queue_size=10)

        # Publish to the Lights
        self.light_state = rospy.Publisher("light_state", String, queue_size = 10)

        self.PORT = '/dev/ttyTHS1'  # CHANGE THIS
        self.ser = serial.Serial(self.PORT, baudrate=115200)
        self.dataout = pynmea2.NMEAStreamReader()
        self.ser.readline() # read one junk line to achieve line synchronization

        self.SET_GPS_NAV = "SGN"
        self.GPS_CODE = "GPS,"

        self.pub = rospy.Publisher('wheel_distance', String, queue_size=10)
        self.outfile = open('/home/autonav/autonav_ws/src/gps_connection/src/gpsData.csv', 'w')
        self.outfile.write('Lat,Long,Curr Heading, Target Heading, Error in Angle,Filtered Error in Angle\n')
        self.LINE_MODE = {self.LINE_FOLLOWING, self.OBJECT_AVOIDANCE_FROM_LINE, self.OBJECT_TO_LINE, self.LINE_TO_OBJECT}
        
        # Subscribe to state updates for the robot
        self.state_sub = rospy.Subscriber("state_topic", String, self.state_callback)

        self.past_loc = self.take_reading()
        self.moving_avg = np.zeros((5,), dtype=np.float32)
        self.moving_avg_idx = 0

        # self.ob_clear_history = np.zeros((3,), dtype=bool)
        # self.ob_idx = 0

        self.done = False
        rospy.loginfo("GPS Node Ready")

    # a 5 point moving average filter
    def filter_angle(self, new_val):
        self.moving_avg = np.roll(self.moving_avg, 1)
        self.moving_avg[0] = new_val
        return sum(self.moving_avg)/self.moving_avg.size
        #self.moving_avg[self.moving_avg_idx] = new_val
        #self.moving_avg_idx = (self.moving_avg_idx + 1) % self.moving_avg.size
        #return np.mean(self.moving_avg)

    # subtracts the angles (x - y) and gives the answer between (-pi, pi]
    def sub_angles(self, x, y):
        a = (x - y + 2 * cmath.pi) % (2 * cmath.pi)
        if a > cmath.pi:
            a -= 2 * cmath.pi
        return a

    # this function calculates the heading in radians from 2 points
    def calc_heading(self, past, curr):
        trajectory = curr - past
        return cmath.phase(trajectory) # in radians [-pi, pi]

    # return true if we can switch back from obstacle avoidance to GPS nav
    def is_object_clear(self, error_angle):
        # self.ob_clear_history[self.ob_idx] = 1 if -0.2 < error_angle * -self.following_dir < 0.2  else 0
        # self.ob_idx = (self.ob_idx + 1) % self.ob_clear_history.size
        # return np.count_nonzero(self.ob_clear_history) >= 4
        return abs(error_angle * -self.following_dir) < self.exit_angle

    # this function returns true if we are within the threshhold of the 
    # GPS waypoint
    def check_waypoint(self, curr):
        target_point = (self.target_loc[self.waypoint_itr].real, self.target_loc[self.waypoint_itr].imag)
        current_point = (curr.real, curr.imag)
        dist_meters = distance.distance(current_point, target_point).m
        rospy.loginfo(dist_meters)
        
        # if we are not currently navigating with GPS data
        if self.state in self.LINE_MODE and False:  #currently completely disabled
            rospy.logerr("Line Follow")
            if dist_meters <= self.TRANS_DIST:
                rospy.loginfo("Arrived")
                self.safe_publish(self.light_state, "G_On")
                self.safe_publish(self.gps_events, "WAYPOINT_FOUND")
                rospy.logerr("Line to GPS")
        # if we are using GPS data to navigate
        else: 
            rospy.logerr("distance: " + str(dist_meters)) 
            rospy.logerr("goal: " + str(self.waypoint_itr))  
            if dist_meters <= self.DISTANCE_GOAL:
                rospy.loginfo("Arrived")
                self.safe_publish(self.light_state, "G_On")
                self.waypoint_itr = (self.waypoint_itr + 1)
                rospy.logerr("Switch points")
                # if we have reached our last waypoint, switch state. 
                if self.waypoint_itr > len(self.target_loc):
                    self.safe_publish(self.gps_events, "WAYPOINT_FOUND")
                    self.waypoint_itr = 0
                    rospy.logerr("End GPS")
        

    # this function takes a measurement and calculates all of the necessary
    # information from the location to send to the motor controller
    def process_gps_data(self):
        loc = self.take_reading() # get the new reading
        # Check if we are at the waypoint
        self.check_waypoint(loc)

        # calculate our current heading and the heading we need to have
        curr_heading = self.calc_heading(self.past_loc, loc)
        target_heading = self.calc_heading(loc, self.target_loc[self.waypoint_itr])
        # calculate and filter the error in the angle of the two headings
        error_angle = self.sub_angles(target_heading, curr_heading)
        filtered_error_angle = self.filter_angle(error_angle)
        rospy.logerr("Current Heading: " + str(curr_heading))
        rospy.logerr("Target Heading: " + str(target_heading))
        rospy.logerr("Error: " + str(filtered_error_angle))


        # check state, if in object avoid and GPS, then check error_angle for release
        if self.state == self.OBJECT_AVOIDANCE_FROM_GPS:
            if self.is_object_clear(filtered_error_angle):
                rospy.logwarn(self.WAYPOINT_STRAIGHT)
                self.safe_publish(self.event_pub, self.WAYPOINT_STRAIGHT)
                # self.event_pub.publish(self.WAYPOINT_STRAIGHT)
                # self.ob_clear_history = np.zeros((3,), dtype=np.float32)

        
        # save off the location
        self.past_loc = loc
        
        # publish
        self.safe_publish(self.pub, self.GPS_CODE + str(filtered_error_angle))
        # self.pub.publish(self.GPS_CODE + str(filtered_error_angle))
        rospy.loginfo("%s,%s,%s,%s\n" % (curr_heading, target_heading, error_angle, filtered_error_angle))
        self.outfile.write("%s,%s,%s,%s\n" % (curr_heading, target_heading, error_angle, filtered_error_angle))

    # this function will read from the gps a single nmea sentence
    # and return a lat, long complex pair
    def take_reading(self):
        newdata=self.ser.readline()
        nmea_type = newdata[1:6]
        try:
            newmsg=pynmea2.parse(newdata)
        except pynmea2.nmea.ChecksumError as e:
            rospy.logerr("Checksum Error")
            rospy.logerr(e)
            return
        except pynmea2.nmea.ParseError as e:
            rospy.logerr("Parsing Error")
            rospy.logerr(e)
            return

        if nmea_type == "GNTXT":
            rospy.logwarn("Possible Error")
            rospy.logwarn(newmsg)
        elif nmea_type == "GNRMC":
            lat=getattr(newmsg, 'latitude', 'N/A')
            lng=getattr(newmsg, 'longitude', 'N/A')
            rospy.loginfo("%s,%s,\n" % (lat, lng))
            self.outfile.write("%s,%s," % (lat, lng))
            return complex(lat, lng)
            #gps_str = "%s: Lat=%s, Long=%s" %(nmea_type, lat, lng)
            #self.pub.publish(gps_str)
        else:
            rospy.logwarn("Unknown Message Type")
            rospy.logwarn(newdata)
        
        return complex(0, 0)

    def spin_gps(self):
        while not(self.done or rospy.is_shutdown()):
            self.process_gps_data()

    def __del__(self):
        rospy.loginfo("Deleting GPS Node")
        rospy.loginfo("Closing GPS Connection")
        
        #close serial port
        self.ser.break_condition = True 
        self.ser.sendBreak(duration = 0.2)
        time.sleep(0.2)
    
        self.ser.close()
        time.sleep(0.2)
        
        # Close the recording file
        self.outfile.close()
        rospy.loginfo("Cleanup Finished")


def main(args):
    g = GPS()
    spin_thread = Thread(target=g.spin_gps)
    
    try:
        spin_thread.start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    spin_thread.join(5)


if __name__ == '__main__':
    main(sys.argv)

