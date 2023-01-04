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
import this
from dataclasses import dataclass
import time
from utils.utils import *
import numpy as np
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
        #self.image_sub         = self.create_subscription(Image,         "/camera/color/image_raw",           self.image_callback,         10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/color/metadata",            self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/depth/camera_info",         self.callback, 10)
        #self.depth_image_sub   = self.create_subscription(Image,         "/camera/depth/image_rect_raw",      self.depth_image_callback,   10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/depth/metadata",            self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/extrinsics/depth_to_color", self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/camera/imu",                       self.callback, 10)
        #self.encoder_sub       = self.create_subscription(EncoderData,   "/encoder_data",                     self.encoder_callback,       10)
        #self.fused_heading_sub = self.create_subscription(HeadingStatus, "/fused_heading",                    self.fused_heading_callback, 10)
        #self.gps_events_sub    = self.create_subscription(String,        "/gps_events",                       self.gps_events_callback,    10)
        #self.gps_heading_sub   = self.create_subscription(HeadingStatus, "/gps_heading",                      self.gps_heading_callback,   10)
        self.lidar_frame_sub   = self.create_subscription(LaserScan,     "/laser_frame",                      self.lidar_frame_callback, 10)
        #self.light_sub         = self.create_subscription(LightCmd,      "/light_events",                     self.light_callback, 10)
        #self.line_sub          = self.create_subscription(String,        "/line_events",                      self.line_callback, 10)
        #self.mod_lidar_sub     = self.create_subscription(String,        "/mod_lidar",                        self.mod_lidar_callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/parameter_events",                 self.callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/rosout",                           self.callback, 10)
        self.lidar_scan_sub   = self.create_subscription(LaserScan,     "/scan",                             self.lidar_scan_callback, 10)
        #self.state_sub        = self.create_subscription(Int32,         "/state_topic",                      self.state_callback, 10)
        #self.x_sub = self.create_subscription(datatype,      "/tf_static",                        self.callback, 10)
        #self.wheel_sub        = self.create_subscription(String,        "/wheel_distance",                   self.wheel_callback, 10)

        t = str(round(time.time()))

        # create & open log files for nodes we've subscribed to
        if hasattr(self, 'image_sub'):
            name               = 'log-' + t + '-image.csv'
            self.image_logfile = open(name, 'w')
            self.image_writer  = csv.writer(self.image_logfile)

        if hasattr(self, 'depth_image_sub'):
            name                     = 'log-' + t + '-depth_image.csv'
            self.depth_image_logfile = open(name, 'w')
            self.depth_image_writer  = csv.writer(self.depth_image_logfile)

        if hasattr(self, 'encoder_sub'):
            name                 = 'log-' + t + '-encoder.csv'
            self.encoder_logfile = open(name, 'w')
            self.encoder_writer  = csv.writer(self.encoder_logfile)

        if hasattr(self, 'fused_heading_sub'):
            name                       = 'log-' + t + '-fused_heading.csv'
            self.fused_heading_logfile = open(name, 'w')
            self.fused_heading_writer  = csv.writer(self.fused_heading_logfile)

        if hasattr(self, 'gps_events_sub'):
            name                    = 'log-' + t + '-gps_events.csv'
            self.gps_events_logfile = open(name, 'w')
            self.gps_events_writer  = csv.writer(self.gps_events_logfile)

        if hasattr(self, 'gps_heading_sub'):
            name                     = 'log-' + t + '-gps_heading.csv'
            self.gps_heading_logfile = open(name, 'w')
            self.gps_heading_writer  = csv.writer(self.gps_heading_logfile)

        if hasattr(self, 'lidar_frame_sub'):
            name                     = 'log-' + t + '-lidar_frame.csv'
            self.lidar_frame_logfile = open(name, 'w')
            self.lidar_frame_writer  = csv.writer(self.lidar_frame_logfile)

        if hasattr(self, 'light_sub'):
            name               = 'log-' + t + '-light.csv'
            self.light_logfile = open(name, 'w')
            self.light_writer  = csv.writer(self.light_logfile)

        if hasattr(self, 'line_sub'):
            name              = 'log-' + t + '-line.csv'
            self.line_logfile = open(name, 'w')
            self.line_writer  = csv.writer(self.line_logfile)

        if hasattr(self, 'mod_lidar_sub'):
            name                   = 'log-' + t + '-mod_lidar.csv'
            self.mod_lidar_logfile = open(name, 'w')
            self.mod_lidar_writer  = csv.writer(self.mod_lidar_logfile)

        if hasattr(self, 'lidar_scan_sub'):
            name                   = 'log-' + t + '-lidar_scan.csv'
            self.lidar_scan_logfile = open(name, 'w')
            self.lidar_scan_writer  = csv.writer(self.lidar_scan_logfile)

        if hasattr(self, 'state_sub'):
            name               = 'log-' + t + '-state.csv'
            self.state_logfile = open(name, 'w')
            self.state_writer  = csv.writer(self.state_logfile)

        if hasattr(self, 'wheel_sub'):
            name               = 'log-' + t + '-wheel.csv'
            self.wheel_logfile = open(name, 'w')
            self.wheel_writer  = csv.writer(self.wheel_logfile)

    ## Callback Functions

    # image_callback function
    # logs rgb image data to csv
    def image_callback(self, data):
        self.image_writer.writerow(np.concatenate(([data.header.stamp.sec, data.header.stamp.nanosec, data.height, data.width], data.data)))
    
    # depth_image_callback function
    # logs depth image data to csv
    def depth_image_callback(self, data):
        self.depth_image_writer.writerow(np.concatenate(([data.header.stamp.sec, data.header.stamp.nanosec, data.height, data.width], data.data)))
    
    # encoder_callback function
    # logs encoder data to csv
    def encoder_callback(self, data):
        # self.get_logger().info(f"left: {data.left}, right: {data.right} ")
        self.encoder_writer.writerow(np.array([data.left, data.right]))

    # lidar_frame_callback function
    # logs lidar frame data to csv
    def lidar_frame_callback(self, data):
        self.lidar_frame_writer.writerow(np.concatenate(([data.header.stamp.sec, data.header.stamp.nanosec], data.ranges)))

    # lidar_scan_callback function
    # logs lidar /scan data to csv
    def lidar_scan_callback(self, data):
        self.lidar_scan_writer.writerow(np.concatenate(([data.header.stamp.sec, data.header.stamp.nanosec], data.ranges)))

    def wheel_callback(self, data):
        msg = data.data.split(',')
        self.wheel_writer.writerow(np.concatenate(([time.time()], np.array(msg))))
        #self.get_logger().info("logged.")

    # destructor
    def __del__(self):
        if hasattr(self, 'image_sub'):
            self.image_logfile.close()

        if hasattr(self, 'depth_image_sub'):
            self.depth_image_logfile.close()

        if hasattr(self, 'encoder_sub'):
            self.encoder_logfile.close()

        if hasattr(self, 'fused_heading_sub'):
            self.fused_heading_logfile.close()

        if hasattr(self, 'gps_events_sub'):
            self.gps_events_logfile.close()

        if hasattr(self, 'gps_heading_sub'):
            self.gps_heading_logfile.close()

        if hasattr(self, 'lidar_frame_sub'):
            self.lidar_frame_logfile.close()

        if hasattr(self, 'light_sub'):
            self.light_logfile.close()

        if hasattr(self, 'line_sub'):
            self.line_logfile.close()

        if hasattr(self, 'mod_lidar_sub'):
            self.mod_lidar_logfile.close()

        if hasattr(self, 'lidar_scan_sub'):
            self.lidar_scan_logfile.close()

        if hasattr(self, 'state_sub'):
            self.state_logfile.close()

        if hasattr(self, 'wheel_sub'):
            self.wheel_logfile.close()


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
