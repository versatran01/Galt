#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from matplotlib import cm
from PyQt4 import QtCore
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import threading
import time


class IntensityNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image', Image, self.image_cb,
                                          queue_size=1)
        plt.ion()
        self.image = None
        self.do_plot = True
        self.thread = threading.Thread(target=self.plot)
        self.thread.start()

    def stop(self):
        self.do_plot = False
        self.thread.join()
        print('Stop thread')

    def plot(self):
        while self.do_plot:
            if self.image is None:
                fig = plt.gcf()
                ax = fig.gca(projection='3d')
                surf = None
            else:
                height, width = self.image.shape
                X = np.arange(0, width, 1)
                Y = np.arange(0, height, 1)
                X, Y = np.meshgrid(X, Y)
                Z = self.image
                if surf is not None:
                    surf.remove()
                surf = ax.plot_surface(X, Y, Z, cstride=2, rstride=2,
                                       cmap=cm.jet)
                plt.draw()
                plt.pause(0.1)

    def image_cb(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
        except CvBridgeError, e:
            print(e)

        self.image = cv2.resize(image, None, fx=0.05, fy=0.05,
                                interpolation=cv2.INTER_NEAREST)


if __name__ == '__main__':
    node = IntensityNode()
    rospy.init_node('intensity_node')
    try:
        rospy.spin()
        node.stop()
    except KeyboardInterrupt:
        print("Shutting down!!!")
