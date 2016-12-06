#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import threading


class IntensityNode(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.transport = 'compressed'
        if self.transport == 'compressed':
            self.image_sub = rospy.Subscriber('image', CompressedImage,
                                              self.image_cb, queue_size=1)
        else:
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
                ax.set_zlim(0, 255)
                surf = None
            else:
                height, width = self.image.shape
                X = np.arange(0, width, 1)
                Y = np.arange(0, height, 1)
                X, Y = np.meshgrid(X, Y)
                Z = self.image
                if surf is not None:
                    surf.remove()
                surf = ax.plot_surface(X, Y, Z, cstride=int(width / 20),
                                       rstride=int(height / 20),
                                       cmap=cm.jet)
                plt.pause(0.01)

    def image_cb(self, image_msg):
        if self.transport == 'compressed':
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
        else:
            try:
                image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')
            except CvBridgeError, e:
                print(e)

        image = cv2.resize(image, None, fx=0.2, fy=0.2,
                           interpolation=cv2.INTER_NEAREST)
        self.image = cv2.GaussianBlur(image, (5, 5), 0)


if __name__ == '__main__':
    node = IntensityNode()
    rospy.init_node('intensity_node')
    try:
        rospy.spin()
        node.stop()
    except KeyboardInterrupt:
        print("Shutting down!!!")
