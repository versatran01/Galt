# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 21:34:34 2016

@author: chao
"""

# %% 
import cv2
import numpy as np
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.blob_analyzer import *
from scpye.visualization import *

# %%
# Raw detection
# red    8
# green  11
color = 'red'
if color == 'red':
    index = 10
    min_area = 9
else:
    index = 11
    min_area = 5
dr = DataReader(color=color, mode='slow_flash')
fd = FruitDetector.from_pickle(dr.model_dir)
I = dr.load_image(index)
bw = fd.detect(I)

# %%
# Raw classifier detection results
min_area = 5
bw = gray_from_bw(bw)
bw = clean_bw(bw)
blobs, cntrs = region_props_bw(bw, min_area=min_area)
bw_clean = fill_bw(bw, cntrs)

disp_left = fd.color
disp_right = fd.color
disp_right[bw_clean == 0] = 0

draw_contours(disp_left, cntrs)
draw_bboxes(disp_right, blobs['bbox'])

imshow2(disp_left, disp_right)

# %%
# Find what is likely to be multiple fruits
disp = fd.color

blobs_multi = []
cntrs_multi = []

areas = blobs['prop'][:, 0]
area_thresh = np.mean(areas)
for blob, cntr in zip(blobs, cntrs):
    bbox = blob['bbox']
    area, aspect, extent = blob['prop']
    if area > area_thresh and (aspect > 1.4 or extent < 0.5):
        draw_contour(disp, cntr, color=(0, 255, 0))
        blobs_multi.append(blob)
        cntrs_multi.append(cntr)
    else:
        draw_contour(disp, cntr)
imshow(disp)

# %%
# For each candidate find max points and split the blob
disp_left = fd.color
v = fd.v
v[bw_clean == 0] = 0
for blob, cntr in zip(blobs_multi, cntrs_multi):
    bbox = blob['bbox']
    bw_bbox = extract_bbox(bw_clean, bbox)
    min_dist = min(np.sqrt(bbox_area(bbox)) / 4.5, 10)
    bgr = extract_bbox(disp_left, bbox)
    image = extract_bbox(v, bbox)
    image_max = ndi.maximum_filter(image, size=3, mode='constant')
    local_max = peak_local_max(image_max, min_distance=min_dist,
                               indices=False, exclude_border=False)
    marker = ndi.label(local_max, structure=np.ones((3, 3)))[0]
    label = watershed(-image_max, marker, mask=bw_bbox)
    local_max = gray_from_bw(local_max)
    points = local_max_points(local_max)
    if points is not None and len(points) > 1:
        draw_points(bgr, points)
        imshow3(bgr, image_max, label)
imshow(disp_left)