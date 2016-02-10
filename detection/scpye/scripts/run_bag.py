# -*- coding: utf-8 -*-
"""
Created on Tue Feb  9 21:39:28 2016

@author: chao
"""

from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.fruit_visualizer import FruitVisualizer

# %%

base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'

dr = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(dr.model_dir)
fv = FruitVisualizer()

for image in dr.load_bag(1):
    fv.show(image, image)
