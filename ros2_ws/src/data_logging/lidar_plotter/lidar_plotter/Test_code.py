import matplotlib.pyplot as plt

ranges = [10, 12, 4.5, 4.7, 10, 11]

def lidar_frame_callback(self, data):
        #self.lidar_frame_writer.writerow(np.concatenate(([data.header.stamp.sec, data.header.stamp.nanosec], data.ranges)))
        
        # build points to plot from scan
        dist_y = np.concatenate(ranges)
        points_x = len(ranges)

        # plotting points as a scatter plot
        plt.scatter(points_x, dist_y, label= "points", color= "green", marker= "*", s=30)
        # axiis, title, display
        plt.xlabel('Samples Per Scan')
        plt.ylabel('Distances (m)')
        plt.title('Lidar laster_frame scan')
        plt.show()
        print("Done")