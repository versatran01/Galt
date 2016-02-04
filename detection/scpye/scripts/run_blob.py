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
def morph_opening(bw, ksize=3):
    """
    http://docs.opencv.org/2.4/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html
    http://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html#gsc.tab=0
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_open = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)
    return bw_open


def morph_closing(bw, ksize=3):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_close = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
    return bw_close


def find_contours(bw):
    """
    http://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html#gsc.tab=0
    """
    cs, _ = cv2.findContours(bw, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)
    return cs


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
