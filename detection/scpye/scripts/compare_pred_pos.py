# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 13:35:01 2016

@author: chao
"""
from __future__ import division
import os
cwd = os.getcwd()
import cv2
from scpye.viz import *
from scpye.blob_analysis import *

img_dir = os.path.join(cwd, '../image')
bw_file = os.path.join(img_dir, 'red_pred.png')
pos_file = os.path.join(img_dir, 'red_pos.png')
bw = cv2.imread(bw_file, cv2.IMREAD_GRAYSCALE)
pos = cv2.imread(pos_file, cv2.IMREAD_GRAYSCALE)

bw_pred = clean_bw(bw)

# %%
blbs_pred, cnts_pred = region_props(bw_pred)
bw_pred = fill_bw(bw_pred, cnts_pred)

# True positive
bw_tp = bw_pred & pos
cnts_tp = find_contours(bw_tp)
n_tp = len(cnts_tp)
print('True positive: {0}'.format(n_tp))

# Relevant elements
cnts_re = find_contours(pos)
n_re = len(cnts_re)
print('Relevent positive: {0}'.format(n_re))

# Recall = True positive / Relevant elements
recall = n_tp / n_re
print('Recall: {0}'.format(recall))

# Selected elements
cnts_se = find_contours(bw_pred)
n_se = len(cnts_se)
print('Selected elements: {0}'.format(n_se))

# Precision = True positive / Selected elements
precision = n_tp / n_se
print('Precision: {0}'.format(precision))

# Visualization
disp = cv2.cvtColor(bw_pred, cv2.COLOR_GRAY2BGR)

# Draw Selected elements, red
draw_contours(disp, cnts_se, color=(255, 0, 0))

# Draw Relevant elements, green
draw_contours(disp, cnts_re, color=(0, 255, 0))

# Draw True positives, cyan
centroids = np.array([contour_centroid(cnt) for cnt in cnts_se])
draw_point(disp, centroids, color=(0, 255, 255), radius=2)
imshow(disp)

