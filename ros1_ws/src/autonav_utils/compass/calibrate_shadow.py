#!/usr/bin/env python

# ********************************************* #
# Cedarville University                         #
# AutoNav Senior Design Team 2020-2021          #
# Obstacle Detection Class                      #
# ********************************************* #

import sys
sys.path.insert(1, '/home/autonav/autonav_ws/src/autonav_utils')
import py_qmc5883l

class Calibrate():

    def __init__(self):
        self.OB_DIST_PARAM = "ObjectStopDistance"
        self.COMP_HEADING_PARAM = "ShadowHeading"

        self.compass = py_qmc5883l.QMC5883L()

        # Read ROS Params
        self.read_param("/DisplayCameraImages", "DISPLAY", False)
        self.read_param("/ObjectCropBottom", "CROP_BOTTOM", 0.40)
        self.read_param("/ObjectCropTop", "CROP_TOP", 0.10)
        self.read_param("/ObjectCropSide", "CROP_SIDE", 0.30)
        self.read_param("/DanielDebug", "DANIEL_DEBUG", False)

        # Connect to RealSense Camera
        self.pipeline = rs.pipeline()
        profile = self.pipeline.start()

        # Sleep to Initialize Camera
        time.sleep(3)

        # Initialize Camera
        self.SCALE = 0.0010000000475
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        if abs(self.SCALE - depth_scale) >= 0.0000000000001:
            rospy.logerr("Depth scale changed to %.13f" % depth_scale)

        # Initialize Colorizer
        self.colorizer = rs.colorizer()

        
    def set_min_dist(self, frames):
        depth_frame = frames.get_depth_frame()

        if self.DISPLAY:
            depth_image = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            cv2.imshow("Original Depth", depth_image)
            cv2.waitKey(2)

        depth = np.asanyarray(depth_frame.get_data(), dtype=np.float64)
        depth = self.crop_depth(depth)

        # Determine Pixel Distances
        distances = depth * self.DEPTH_SCALE
        distances[distances == 0] = 10

        self.distance = np.min(distances)
        if self.is_debug: print("Distance: {:.2f}".format(self.distance))

    def crop_depth(self, depth):
        y, x = depth.shape[0], depth.shape[1]
        return depth[int(y*self.CROP_TOP):-int(y*self.CROP_BOTTOM), int(x*self.CROP_SIDE):-int(x*self.CROP_SIDE)]

    def set_heading(self):
        self.heading = compass.get_bearing()
        if self.is_debug: print("Heading: {:.2f}".format(self.heading))

    def run(self):
        while True:
            frames = self.pipeline.wait_for_frames()
            self.set_min_dist(frames)
            self.set_heading()

    def set_calibration_params(self):
        lines = []
        with open('/home/autonav/autonav_ws/params/params.yaml', 'r') as f:
            lines = f.readlines()
        with open('/home/autonav/autonav_ws/params/calibrate_params.yaml', 'w') as f:
            for line in lines:
                if line[:len(self.OB_DIST_PARAM)] == self.OB_DIST_PARAM:
                    line = line.split(":")[0] + " " + self.distance + "\n"
                elif line[:len(self.COMP_HEADING_PARAM)] == self.COMP_HEADING_PARAM:
                    line = line.split(":")[0] + " " + self.heading + "\n"
                f.write(line)

def main(args):
    c = Calibrate()
    try:
        c.run()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt")
        c.set_calibration_params()

if __name__ == "__main__":
    main(sys.argv)
