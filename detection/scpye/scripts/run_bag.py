# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 21:39:28 2016

@author: chao
"""

import matplotlib.pyplot as plt
import rosbag 
from cv_bridge import CvBridge, CvBridgeError
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector

# %%
def ScpyeVisualizer(object):
    def __init__(self):
        pass


# %%
bridge = CvBridge()
h_color = None
h_bw = None

fig = plt.figure()
plt.ion()

base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'

drd = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(drd.model_dir)
via = ScpyeVisualizer()

with rosbag.Bag(bagfile) as bag:
    for i, (topic, msg, t) in enumerate(bag.read_messages(im_topic)):
        try:
            image = bridge.imgmsg_to_cv2(msg)
            # Rotate image 90 degree

        except CvBridgeError as e:
            print(e)
            continue
        
        
#        fruits, bw = fd.detect(I)
        if h_bgr:
            h_bw.set_data(mask)
            h_bgr.set_data(disp)
        else:
            h_bw = ax_bw.imshow(mask)
            h_bgr = ax_bgr.imshow(disp)
        plt.pause(0.001)