################################
# AutoNav 2023 Competition Robot
# Package: logging
# File: logger.py
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

class Logger(Node):
    def __init__(self):
        super().__init__('logger')

        # Subscribe to nodes you'd like data from
        self.x_sub = self.create_subscription(datatype, "topic name", self.x_callback, 10)

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
    logger = Logger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
