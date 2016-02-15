# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 21:34:34 2016

@author: chao
"""

# %% 
import os
import cv2
import numpy as np
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.region_props import find_contours
from scpye.blob_analyzer import *
from scpye.visualization import *

save_dir = '/home/chao/Desktop'

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

#cv2.imwrite(os.path.join(save_dir, color + '_raw.png'), fd.color)

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

draw_contours(disp_left, cntrs, color=(0, 0, 255))
draw_bboxes(disp_right, blobs['bbox'], color=(0, 0, 255))

imshow2(disp_left, disp_right)
#cv2.imwrite(os.path.join(save_dir, color + '_cntr.png'), disp_left)
#cv2.imwrite(os.path.join(save_dir, color + '_bbox.png'), disp_right)

# %%
# Find what is likely to be multiple fruits
disp = fd.color

blobs_multi = []
cntrs_multi = []
cntrs_single = []
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
        draw_contour(disp, cntr, color=(0, 0, 255))
        cntrs_single.append(cntr)
imshow(disp)
#cv2.imwrite(os.path.join(save_dir, color + '_multi.png'), disp)

# %%
# For each candidate find max points and split the blob
disp_left = fd.color
draw_contours(disp_left, cntrs_single, color=(0, 0, 255))
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
    n = np.max(label)
    local_max = gray_from_bw(local_max)
    points = local_max_points(local_max)
    if points is not None and len(points) > 1:
        for i in np.arange(1, n + 1):
            mask = np.array(label == i, np.uint8)
            cs = find_contours(mask)
            draw_contour(bgr, cs[0], color=(0, 255, 255))
                        
            
        draw_points(bgr, points, radius=2, color=(0, 255, 255))
        
imshow(disp_left)
cv2.imwrite(os.path.join(save_dir, color + '_split.png'), disp_left)