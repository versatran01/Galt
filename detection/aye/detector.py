from __future__ import print_function, division, absolute_import
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from sklearn.externals import joblib
from aye.preprocessing import *

bagfile = '/home/chao/Workspace/bag/frame/rect_fixed/frame1_rect_fixed.bag'

# load
clf = joblib.load('../model/ensemble.pkl')
scaler = joblib.load('../model/scaler.pkl')
im_topic = '/color/image_rect_color'

bridge = CvBridge()
fig = plt.figure()
ax_bgr = fig.add_subplot(121)
ax_bw = fig.add_subplot(122)
h_bgr = None
h_bw = None
with rosbag.Bag(bagfile) as bag:
    for i, (topic, msg, t) in enumerate(bag.read_messages(im_topic)):
        try:
            im_bgr = bridge.imgmsg_to_cv2(msg)
            # Rotate image 90 degree
            im_bgr = cv2.transpose(im_bgr)
            im_bgr = cv2.flip(im_bgr, 1)

        except CvBridgeError as e:
            print(e)

        s = Samples(im_bgr)
        X = scaler.transform(s.X())

        y = clf.predict(X)
        bw = s.y_to_bw(y, to_gray=True)

        n = 3
        kernel = np.ones((n, n), np.uint8)
        opened = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel)

        if h_bgr:
            h_bw.set_data(opened)
            h_bgr.set_data(s.im_bgr)
        else:
            h_bw = ax_bw.imshow(bw, cmap=plt.cm.Greys)

            #convert to rgb
            b, g, r = cv2.split(s.im_bgr)
            rgb_img = cv2.merge([r, g, b])

            h_bgr = ax_bgr.imshow(rgb_img)
        plt.pause(0.01)
        plt.draw()
