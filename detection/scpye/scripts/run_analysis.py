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

# %%
def load_counts(base_dir, color, mode, side, index):
    dr = DataReader(base_dir, color=color, mode=mode, side=side)
    return dr.load_count(index)

def counts_per_tree(counts, num_trees):
    counts = counts[2:]
    counts_per_tree = np.array_split(counts, num_trees)
    return np.array([sum(e) for e in counts_per_tree])
    
# %%
# Read ground truth counts
dr = DataReader(base_dir, color=color, mode=mode)
frame1_counts_gt = dr.load_ground_truth()
frame1_total_gt = np.sum(frame1_counts_gt)
num_trees = len(frame1_counts_gt) 
frame_trees = np.arange(1, num_trees + 1)

k_north = 0.75
k_south = 1 - k_north

calib_totals = []
north_totals = []
south_totals = []

# For each frame
f, axarr = plt.subplots(2, 2, figsize=(13, 9))
for i, ax in enumerate(axarr.ravel()):
    n_frame = i + 1
    north_counts = load_counts(base_dir, color, mode, 'north', n_frame)
    south_counts = load_counts(base_dir, color, mode, 'south', n_frame)
    north_total = np.sum(north_counts)
    south_total = np.sum(south_counts)
    if i == 0:
        # Get factor k
        k = frame1_total_gt / (k_north * north_total + k_south * south_total)
        ax.plot(frame_trees, frame1_counts_gt, color='b', label='truth')
    
    north_counts_per_tree = counts_per_tree(north_counts, num_trees)
    south_counts_per_tree = counts_per_tree(south_counts, num_trees)
    calib_counts_per_tree = (k_north * north_counts_per_tree + \
                             k_south * south_counts_per_tree) * k  
    
    calib_total = (k_north * north_total + k_south * south_total) * k
    
    ax.plot(frame_trees, calib_counts_per_tree, color='g', label='calib')
    ax.set_xlabel('trees')
    ax.set_ylabel('fruits')
    ax.set_title('{0} rep {1}'.format(color, n_frame))
    ax.grid(True)
    ax.set_xlim([0, num_trees + 1])
    
    if i == 0:
        ax.legend(ncol=2, mode='expand')
    north_totals.append(north_total)
    south_totals.append(south_total)
    calib_totals.append(calib_total)

north_totals = np.array(north_totals, np.int)   
south_totals = np.array(south_totals, np.int)
calib_totals = np.array(calib_totals, np.int)

# %%
fig = plt.figure()
ax = fig.add_subplot(111)
x = np.arange(4) * 1.5
bar_width = 0.4

ax.set_title(color)
ax.set_ylabel('fruits')
ax.set_xticks(x + bar_width)
ax.set_xticklabels(('rep1', 'rep2', 'rep3', 'rep4'))
ax.set_xlim([x[0] - bar_width, x[-1] + bar_width * 4])
ax.set_ylim([0, 3500])
ax.grid(True)
rects_north = ax.bar(x, north_totals, bar_width, color='r')
rects_south = ax.bar(x + bar_width, south_totals, bar_width, color='y')
rects_calib = ax.bar(x + bar_width * 2, calib_totals, bar_width)
for rect in rects_calib:
    height = rect.get_height()
    ax.text(rect.get_x() + rect.get_width() / 2, 1.01 * height, '%d' % int(height),
            ha='center', va='bottom')
ax.legend((rects_north[0], rects_south[0], rects_calib), ('North', 'South', 'Calib'),
          mode='expand', ncol=3)