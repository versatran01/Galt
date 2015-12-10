from __future__ import print_function, division, absolute_import
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt

from sklearn.externals import joblib
from aye.preprocessing import Samples

bagfile = '/home/chao/Workspace/bag/frame/rect_fixed/frame1_rect_fixed.bag'

# load
eclf = joblib.load('../model/ensemble.pkl')

im_topic = '/color/image_rect_color'

bridge = CvBridge()
fig = plt.figure()
ax_bgr = fig.add_subplot(121)
ax_bw = fig.add_subplot(122)
h_bgr = None
h_bw = None
with rosbag.Bag(bagfile) as bag:
    for topic, msg, t in bag.read_messages(im_topic):
        try:
            im_bgr = bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
        s = Samples(im_bgr)
        X = s.X()
        y = eclf.predict(X)
        bw = s.y_to_bw(y)

        if h_bgr:
            h_bw.set_data(bw)
            h_bgr.set_data(s.im_bgr)
        else:
            h_bw = ax_bw.imshow(bw, cmap=plt.cm.Greys)
            h_bgr = ax_bgr.imshow(s.im_bgr)
        plt.pause(0.01)
        plt.draw()
