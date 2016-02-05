# -*- coding: utf-8 -*-
"""
Created on Thu Feb  4 14:30:18 2016

@author: chao
"""

import os
import cv2
import numpy as np
from scpye.viz import *

# %%
cwd = os.getcwd()
bgr_file = os.path.join(cwd, '../image/red_bgr.png')
bw_file = os.path.join(cwd, '../image/red_bw.png')
img_bgr = cv2.imread(bgr_file, cv2.IMREAD_COLOR)
img_bw = cv2.imread(bw_file, cv2.IMREAD_GRAYSCALE)


# %%

def regionprops_cv(bw, image=None):
    cs = find_contours(bw.copy())

    for cnt in cs:
        m = cv2.moments(cnt)
        area = m['m00']
        if area > 0:
            bbox = cv2.boundingRect(cnt)
            bbox_area = bbox[-1] * bbox[-2]
            extent = area / bbox_area
            equiv_diameter = np.sqrt(4 * area / np.pi)
            # Poly
            epsilon = 0.1 * cv2.arcLength(cnt, True)
            center, radius = cv2.minEnclosingCircle


def regionprops_sk(bw, image=None):
    pass


# %%
bw = img_bw
bw = morph_opening(bw)
bw = morph_closing(bw)
imshow2(img_bgr, bw)

# Contour
disp = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)
cs = find_contours(bw.copy())
# Poly
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
        solidity = area / cvx_area

        # Ellipse
        center, axes, angle = cv2.fitEllipse(cnt)
        ellipse = np.hstack((center, axes, angle))
        draw_ellipse(disp, ellipse)

        MAJ = np.argmax(axes)
        maj_axes = axes[MAJ]
        min_axes = axes[1 - MAJ]
        eccen = np.sqrt(1 - (min_axes / maj_axes) ** 2)
        draw_contour(disp, cnt)
        draw_text(disp, solidity * 100, center)
        
imshow(disp, fsize=(15, 15))
