#!/usr/bin/env python
import roslib
roslib.load_manifest('led')
import rospy
from sensor_msgs.msg import Image
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt

class intensity_profiler:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_callback)
        cv2.namedWindow("Live Image", 1)
        plt.ion()

    def image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #get image information
        height, width, channels = cv_image.shape;
        #convert to np array
        np_image = np.asarray(cv_image_gray)

        #find max values and draw lines on image
        itemindex = np.where(np_image == np.amax(np_image))
        x_max = int(np.median(itemindex[1]))
        y_max = int(np.median(itemindex[0]))

        vertical = rospy.get_param("~vertical")

        plt.cla();

        if (vertical == True):
            plt.plot(np_image[:,x_max])
            cv2.line(cv_image,(x_max,0),(x_max,height),(51,51,255),2)
        else:
            plt.plot(np_image[y_max,:])
            cv2.line(cv_image,(0,y_max),(width,y_max),(51,51,255),2)

        plt.ylabel("Intensity")
        plt.xlabel("Distance on Profile (p)")
        plt.title("Intensity Profile")

        fig = plt.gcf()
        fig.canvas.set_window_title('Intensity Profile')

        plt.draw()
        plt.pause(1.0/1000)

        #show image
        cv2.imshow("Live Image", cv_image)
        cv2.waitKey(1)

def main(args):
    profiler = intensity_profiler()
    rospy.init_node('intensity_profiler')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)