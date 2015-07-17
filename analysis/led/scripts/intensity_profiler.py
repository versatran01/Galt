#!/usr/bin/env python
from __future__ import print_function
import rospy
import roslib
roslib.load_manifest('led')
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt


class IntensityProfiler:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        self.image_pub = rospy.Publisher("intensity_profiler/image_drawn",Image,queue_size=1)
        plt.ion()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print(e)

        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # get image information
        height, width, channels = cv_image.shape;
        # convert to np array
        np_image = np.asarray(cv_image_gray)
        np_image[0:15,:] = 0 #to get rid of bad pixels

        # find max values and draw lines on image
        itemindex = np.where(np_image == np.amax(np_image))
        x_max = int(np.median(itemindex[1]))
        y_max = int(np.median(itemindex[0]))

        #plot and draw line on image
        vertical = rospy.get_param("~vertical")
        plt.cla()
        if vertical:
            plt.plot(np_image[:, x_max])
            cv2.line(cv_image, (x_max, 0), (x_max, height), (51, 51, 255), 2)
        else:
            plt.plot(np_image[y_max, :])
            cv2.line(cv_image, (0, y_max), (width, y_max), (51, 51, 255), 2)
        plt.ylabel("Intensity")
        plt.xlabel("Distance on Profile (p)")
        plt.ylim([0, 255])
        plt.title("Intensity Profile")
        fig = plt.gcf()
        fig.canvas.set_window_title('Intensity Profile')
        plt.draw()

        #publish
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError, e:
            print(e)

def main(args):
    profiler = IntensityProfiler()
    rospy.init_node('intensity_profiler')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
