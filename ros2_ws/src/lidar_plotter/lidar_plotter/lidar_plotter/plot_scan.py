################################
# AutoNav 2023 Competition Robot
# Package       : lidar_plotter
# File          : data_logger.py
# Purpose       : create and plot a csv file with data from lidar 
#                 laser_frame topic subscribed to
# Date Created  : 24 Jan 2023
# Date Modified : 24 Jan 2023
################################

from dataclasses import dataclass
import time
#from utils.utils import *
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node      import Node
from custom_msgs.msg import *
from std_msgs.msg    import *
from sensor_msgs.msg import *

class Plot_Scan(Node):
    def __init__(self):
        super().__init__('Plot_Scan')

        # Subscribe to node
        self.lidar_frame_sub = self.create_subscription(LaserScan, "/laser_frame", self.lidar_frame_callback, 10)

        #record the time for time stamps
        t = str(round(time.time()))

    # lidar_frame_callback function
    # logs lidar frame data to csv
    def lidar_frame_callback(self, data):
        
        self.get_logger().info(data.ranges)
        self.get_logger().info(len(data.ranges))
        # build points to plot from scan
        dist_y = data.ranges
        points_x = np.arange(1, len(data.ranges)+1, 1)

        # plotting points as a scatter plot
        plt.scatter(points_x, dist_y, label= "points", color= "green", marker= "*", s=30)
        # axiis, title, display
        plt.xlabel('Samples Per Scan')
        plt.ylabel('Distances (m)')
        plt.title('Lidar laster_frame scan')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plot_scan = Plot_Scan()
    try:
        rclpy.spin(plot_scan)
    except KeyboardInterrupt:
        plot_scan.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
