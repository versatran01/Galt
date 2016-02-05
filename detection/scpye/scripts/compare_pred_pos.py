# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 13:35:01 2016

@author: chao
"""

import os
cwd = os.getcwd()
import cv2
from scpye.viz import *
from scpye.blob import find_contours, clean_bw

img_dir = os.path.join(cwd, '../image')
bw_file = os.path.join(img_dir, 'red_pred.png')
pos_file = os.path.join(img_dir, 'red_pos.png')
bw = cv2.imread(bw_file, cv2.IMREAD_GRAYSCALE)
pos = cv2.imread(pos_file, cv2.IMREAD_GRAYSCALE)

bw = clean_bw(bw)
imshow2(bw, pos)

disp = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)
# %%
cs = find_contours(bw.copy())
for cnt in cs:
    m = cv2.moments(cnt)
    area = m['m00']
    if area >= 25:
        
        bbox = np.array(cv2.boundingRect(cnt))
#        draw_bbox(disp, bbox)

        bbox_area = bbox[-1] * bbox[-2]
        extent = area / bbox_area
        equiv_diameter = np.sqrt(4 * area / np.pi)

        # Cricle
#        center, radius = cv2.minEnclosingCircle(cnt)
#        circle = np.hstack((center, radius))
#        draw_circle(disp, circle)

        # Convex
        cvx_hull = cv2.convexHull(cnt)
        cvx_area = cv2.contourArea(cvx_hull)
        # Solidity
        solid = area / cvx_area

        # Ellipse
        center, axes, angle = cv2.fitEllipse(cnt)
        ellipse = np.hstack((center, axes, angle))

        MAJ = np.argmax(axes)
        maj_axes = axes[MAJ]
        min_axes = axes[1 - MAJ]
        axes_ratio = min_axes / maj_axes
        # Eccentricity
        eccen = np.sqrt(1 - axes_ratio ** 2)
        
        #draw_ellipse(disp, ellipse)
        draw_contour(disp, cnt)
        draw_text(disp, solid * 100, center)

imshow(disp)

# TODO: Given bw and pos