from __future__ import print_function, division, absolute_import

import os

import cv2
import matplotlib.pyplot as plt
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from sklearn.externals import joblib

from scpye.blob import region_props
from scpye.fruit_detector import FruitDetector
from scpye.fruit_tracker import FruitTracker
from scpye.viz import draw_bboxes

k = 0.3
apple = 'green'
if apple == 'green':
    roi = [240, 200, 1440, 800]
else:
    roi = [0, 200, 1440, 800]
frame_dir = 'frame_' + apple
model_dir = '../model/' + apple

# Data to process
im_topic = '/color/image_rect_color'
bagfile = '/home/chao/Workspace/bag/' + frame_dir + \
          '/rect_fixed/frame2_rect_fixed.bag'

# Load learning stuff
clf = joblib.load(os.path.join(model_dir, 'svc.pkl'))
scaler = joblib.load(os.path.join(model_dir, 'scaler.pkl'))

# Detector and Tracker
detector = FruitDetector(clf, scaler, roi, k)
tracker = FruitTracker()

# Ros
bridge = CvBridge()

# Visualization
fig = plt.figure()
plt.ion()
ax_bgr = fig.add_subplot(121)
ax_bw = fig.add_subplot(122)
h_bgr = None
h_bw = None

# Main counting loop
with rosbag.Bag(bagfile) as bag:
    for i, (topic, msg, t) in enumerate(bag.read_messages(im_topic)):
        try:
            image = bridge.imgmsg_to_cv2(msg)
            # Rotate image 90 degree

        except CvBridgeError as e:
            print(e)
            continue

        # Detection and tracking
        s, bw = detector.detect(image)
        blobs, bw = region_props(bw)
        tracker.track(s, blobs, bw)

        # Visualize result
        disp = tracker.disp
        mask = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)
        bboxes = blobs['bbox']
        draw_bboxes(mask, bboxes, color=(255, 0, 0))

        if h_bgr:
            h_bw.set_data(mask)
            h_bgr.set_data(disp)
        else:
            h_bw = ax_bw.imshow(mask)
            h_bgr = ax_bgr.imshow(disp)
        plt.pause(0.001)
        print(tracker.total_counts)

tracker.finish()
print(tracker.total_counts)
