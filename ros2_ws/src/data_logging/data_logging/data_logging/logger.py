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

class Data_Logger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Subscribe to nodes you'd like data from
        self.x_sub = self.create_subscription(datatype, "topic name", self.x_callback, 10)
        self.x_sub = self.create_subscription(datatype, "/camera/color/camera_info"
	self.x_sub = self.create_subscription(datatype, "/camera/color/image_raw"
	self.x_sub = self.create_subscription(datatype, "/camera/color/metadata
	self.x_sub = self.create_subscription(datatype, "/camera/depth/camera_info
	self.x_sub = self.create_subscription(datatype, "/camera/depth/image_rect_raw
	self.x_sub = self.create_subscription(datatype, "/camera/depth/metadata
	/camera/extrinsics/depth_to_color
	/camera/imu
	/encoder_data
	/fused_heading
	/gps_events
	/gps_heading
	/laser_frame
	/light_events
	/line_events
	/mod_lidar
	/parameter_events
	/rosout
	/scan
	/state_topic
	/tf_static
	/wheel_distance


        # create & open log file
        if hasattr(self, 'x_sub'):
            name         = 'x-log-' + str(round(time.time())) + '.csv'
            self.x_logfile = open(name, 'w')
            self.x_writer  = csv.writer(self.x_logfile)

    # x_callback function
    # logs x data to csv
    def x_callback(self, data):
        self.x_writer.writerow([str(round(time.time())), data])


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
