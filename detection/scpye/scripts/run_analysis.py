# -*- coding: utf-8 -*-
"""
Created on Sat Feb 13 12:43:40 2016

@author: chao
"""
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from scpye.data_reader import DataReader

base_dir = '/home/chao/Workspace/bag'
color = 'green'
mode = 'slow_flash'

dr = DataReader(base_dir, color=color, mode=mode, side='south')
    

# %%
# Total count of 4 frames
frame1_counts_gt = dr.load_ground_truth()
num_trees = len(frame1_counts_gt) 
frame_trees = np.arange(1, num_trees + 1)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(frame_trees, frame1_counts_gt)
ax.set_xlabel('trees')
ax.set_ylabel('fruits')
ax.set_title('frame 1')
ax.set_xlim([0, num_trees + 1])

frame1_counts_total_gt = np.sum(frame1_counts_gt)
frame1_counts = dr.load_count(1)
frame1_total = np.sum(frame1_counts)
k = frame1_counts_total_gt / frame1_total
frame1_counts_per_tree = np.array_split(frame1_counts, num_trees)
frame1_counts_per_tree = np.array([sum(e) for e in frame1_counts_per_tree])
ax.plot(frame_trees, frame1_counts_per_tree * k)
print(frame1_total)

# %%
num_frames = 4
frame_indices = np.arange(1, num_frames + 1)
bar_width = 0.2

north_counts = []
south_counts = []

dr = DataReader(base_dir, color=color, mode=mode, side='north')
for ind in frame_indices:
    frame_counts = dr.load_count(ind)
    frame_total = np.sum(frame_counts)
    north_counts.append(frame_total)
north_counts = np.array(north_counts)

dr = DataReader(base_dir, color=color, mode=mode, side='south')
for ind in frame_indices:
    frame_counts = dr.load_count(ind)
    frame_total = np.sum(frame_counts)
    south_counts.append(frame_total)
south_counts = np.array(south_counts)

frame1_truth_counts = dr.load_ground_truth()
frame1_truth_total = np.sum(frame1_truth_counts)
k = frame1_truth_total * 2.0 / (north_counts[0] + south_counts[0])

calib_counts = []
for n, s in zip(north_counts, south_counts):
    c = (n + s) / 2.0 * k
    calib_counts.append(c)
calib_counts = np.array(calib_counts, np.int)

fig, ax = plt.subplots()
x = np.arange(num_frames)
rects_north = ax.bar(x, north_counts, bar_width, color='r')
rects_south = ax.bar(x + bar_width, south_counts, bar_width, color='y')
rects_calib = ax.bar(x + bar_width * 2, calib_counts, bar_width, color='b')