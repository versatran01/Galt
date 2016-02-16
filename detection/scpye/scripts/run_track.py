# -*- coding: utf-8 -*-
"""
Created on Mon Feb 15 13:03:10 2016

@author: chao
"""

import cv2
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.blob_analyzer import BlobAnalyzer
from scpye.fruit_tracker import FruitTracker
from scpye.fruit_visualizer import FruitVisualizer

base_dir = '/home/chao/Workspace/bag'
color = 'green'
mode = 'slow_flash'
side = 'north'
bag_ind = 1
min_area = 12

dr = DataReader(base_dir, color=color, mode=mode, side=side)
fd = FruitDetector.from_pickle(dr.model_dir)
ba = BlobAnalyzer(split=False, min_area=min_area)
ft = FruitTracker(min_age=3, max_level=4)
image_dir = '/home/chao/Desktop/' + color

fv = FruitVisualizer(image_dir=image_dir)

i = 0
for image in dr.load_bag(bag_ind):
    bw = fd.detect(image)
    fruits, bw_clean = ba.analyze(bw, fd.v)
    ft.track(fd.color, fruits)
    fv.show(ft.disp, bw_clean)
    i += 1
    if i == 20:
        break
