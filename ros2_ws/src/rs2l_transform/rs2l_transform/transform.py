################################
# AutoNav 2022 Competition Robot
# Package: rs2l_transform
# File: transform.py
# Purpose: takes lidar and real sense camera messages and maps real sense onto lidar then republishes to new topic
# Author: Rachael Judy
# Date Modified: 10 May 2022
################################

import rclpy
from rclpy.node import Node

from sensor_msgs import LaserScan
from sensor_msgs import Image

class LidarPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'mlidar', 10)
#        timer_period = 0.5  # seconds
#        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg.data = 'Hello World: %d' % self.i                                  
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
                                             
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
