################################
# AutoNav 2023 Competition Robot
# Package       : lidar_plotter
# File          : data_logger.py
# Purpose       : create and plot a csv file with data from lidar 
#                 laser_frame topic subscribed to
# Date Created  : 24 Jan 2023
# Date Modified : 24 Jan 2023
################################
import time

import numpy
from utils.utils import *
from matplotlib import pyplot as plt
import numpy as np
from numpy import inf
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
        # use /scan if you want to see everything that the lidar sees
        #self.lidar_frame_sub = self.create_subscription(LaserScan, "/scan", self.lidar_frame_callback, 10)

        #record the time for time stamps
        t = str(round(time.time()))

    # lidar_frame_callback function
    # logs lidar frame data to csv
    def lidar_frame_callback(self, data):
        
        #self.get_logger().info(f"Data ranges: {data.ranges} \n Len of ranges: {len(data.ranges)} \n")
        # build points to plot from scan
        dist_y = np.array(data.ranges)
        #dist_y[dist_y == inf] = inf

        # if you want to see the lidar plot in ft
        #dist_y = np.array(dist_y) * 3.28084

        points_x = np.arange(1, len(data.ranges)+1, 1)
        # make lidar right/left is same as plot right/left - note: it doesn't come in this way
        points_x = np.flip(points_x)
        #self.get_logger().info(f"dist_y: {dist_y}")

        #windowMax = self.get_parameter('/ObstacleToPlainDistance').value
        #windowMin = self.get_parameter('/ObstacleNoiseMinDist').value

        # plot only the useful data in a certain window
        for i in range(len(dist_y)):
            # only look in the window that we care about
            if (dist_y[i] > 2) or (dist_y[i] < 0.2): #(i < 4) or (i > (len(dist_y) - 4)) or
                dist_y[i] = 3 # changed from 0 to 3
        # plotting points as a scatter plot
        plt.scatter(points_x, dist_y, label= "points", color= "black", marker= ".", s=30)

        # axiis, title, display
        plt.xlabel('Samples Per Scan')
        plt.ylabel('Distances (m)')
        # if you want distance in ft but also change the comment above
        #plt.ylabel('Distances (ft)')
        plt.title('Lidar laster_frame scan')
        plt.show()
        # you can also press ctl C to close the program else it will keep popping up
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    plot_scan = Plot_Scan()
    try:
        rclpy.spin(plot_scan)
    except KeyboardInterrupt:
        plot_scan.destroy_node()
        plot_scan.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
