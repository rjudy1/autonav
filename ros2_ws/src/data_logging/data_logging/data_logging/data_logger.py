################################
# AutoNav 2023 Competition Robot
# Package: data_logging
# File: data_logger.py
# Purpose: create a csv file with data from topics subscribed to
# Date Created:  1 Oct 2023
# Date Modified: 1 Oct 2023
################################

import cmath
import csv
from dataclasses import dataclass
import time
from utils.utils import *

import rclpy
from rclpy.node      import Node
from custom_msgs.msg import *
from std_msgs.msg    import *
from sensor_msgs.msg import *

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Subscribe to nodes you'd like data from. comment out nodes you don't want.
        #self.x_sub = self.create_subscription(datatype,      "/camera/color/camera_info",         self.callback, 10)
        self.image_sub         = self.create_subscription(Image,         "/camera/color/image_raw",           self.image_callback,         10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/color/metadata",            self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/depth/camera_info",         self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/depth/image_rect_raw",      self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/depth/metadata",            self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/extrinsics/depth_to_color", self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/imu",                       self.callback, 10)
        #self.encoder_sub       = self.create_subscription(EncoderData,   "/encoder_data",                     self.encoder_callback,       10)
        #self.fused_heading_sub = self.create_subscription(HeadingStatus, "/fused_heading",                    self.fused_heading_callback, 10)
        #self.gps_events_sub    = self.create_subscription(String,        "/gps_events",                       self.gps_events_callback,    10)
        #self.gps_heading_sub   = self.create_subscription(HeadingStatus, "/gps_heading",                      self.gps_heading_callback,   10)
        #self.lidar_frame_sub   = self.create_subscription(LaserScan,     "/laser_frame",                      self.lidar_frame_callback, 10)
        #self.light_sub         = self.create_subscription(LightCmd,      "/light_events",                     self.light_callback, 10)
        #self.line_sub          = self.create_subscription(String,        "/line_events",                      self.line_callback, 10)
        #self.mod_lidar_sub     = self.create_subscription(String,        "/mod_lidar",                        self.mod_lidar_callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/parameter_events",                 self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/rosout",                           self.callback, 10)
        #self.lidar_scan_sub   = self.create_subscription(LaserScan,     "/scan",                             self.lidar_scan_callback, 10)
        #self.state_sub        = self.create_subscription(Int32,         "/state_topic",                      self.state_callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/tf_static",                        self.callback, 10)
        #self.wheel_sub        = self.create_subscription(String,        "/wheel_distance",                   self.wheel_callback, 10)

        t = str(round(time.time()))

        # create & open log files for nodes we've subscribed to
        if hasattr(self, 'image_sub'):
            name               = 'image-log-' + t + '.csv'
            self.image_logfile = open(name, 'w')
            self.image_writer  = csv.writer(self.image_logfile)

        if hasattr(self, 'encoder_sub'):
            name               = 'encoder-log-' + t + '.csv'
            self.encoder_logfile = open(name, 'w')
            self.encoder_writer  = csv.writer(self.encoder_logfile)

    # image_callback function
    # logs image data to csv
    def image_callback(self, data):
        self.image_writer.writerow([str(round(time.time())), data])


def main(args=None):
    rclpy.init(args=args)
    data_logger = Data_Logger()
    try:
        rclpy.spin(data_logger)
    except KeyboardInterrupt:
        data_logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
