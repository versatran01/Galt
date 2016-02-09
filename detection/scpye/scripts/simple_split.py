# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 20:45:49 2016

@author: chao
"""

# %%
from __future__ import division
from scpye.bounding_box import extract_bbox
from scpye.region_props import *
from scpye.blob_analyzer import *
from scpye.testing import get_prediction_bw
from scpye.training import *
from scpye.visualization import *
import scipy.ndimage as ndi

# %%
base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'
test_indices = [11]

def num_local_maximas(image, n=8):
    """
    http://answers.opencv.org/question/28035/find-local-maximum-in-1d-2d-mat/
    :param image:
    :param n:
    :return:
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    peak = cv2.dilate(image, kernel, iterations=1)
    peak -= image

    flat = cv2.erode(image, kernel, iterations=1)
    flat = image - flat

    peak[peak > 0] = 255
    flat[flat > 0] = 255

    flat = cv2.bitwise_not(flat)
    peak[flat > 0] = 255
    peak = cv2.bitwise_not(peak)

    return peak

def loca_max_point(image):
    """
    http://answers.opencv.org/question/28035/find-local-maximum-in-1d-2d-mat/
    :param image:
    :param n:
    :return:
    """
    cs, _ = cv2.findContours(image, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_NONE)
                             
    centroids = []                        
    for cnt in cs:
        m = cv2.moments(cnt)
        a = cv2.contourArea(cnt)
        if a > 0 :
            c = np.array((m['m10'] / m['m00'], m['m01'] / m['m00']))
            centroids.append(c)
    centroids = np.array(centroids)
    return centroids
    
# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

Is, Ls = drd.load_image_label_list(test_indices)

#for I, L in zip(Is, Ls):
#    B = get_prediction_bw(img_ppl, img_clf, I)
#    B = gray_from_bw(B)   
#    B = clean_bw(B)
#
#    blobs, cntrs = region_props_bw(B, min_area=5)
#    B = fill_bw(B, cntrs)
#    
#    disp_bgr = img_ppl.named_steps['remove_dark'].image
#    v = img_ppl.named_features['hsv'].image[:,:,-1]
#    
#    areas = blobs['prop'][:, 0]
#    area_thresh = np.mean(areas)
#    bboxes = []
#    for blob in blobs:
#        bbox = blob['bbox']
#        prop = blob['prop']
#        area, aspect, extent = blob['prop']
#        if area > area_thresh and (aspect > 1.5 or extent < 0.6):
#            label, n = label_blob(bbox, B, v, k=5.5, return_num=True)
#            if n == 1:
#                draw_bbox(disp_bgr, bbox, color=(0, 0, 255))
#            else:
#                draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
#                imshow2(extract_bbox(disp_bgr, bbox), label)
#        else:
#            bboxes.append(bbox)
#    bboxes = np.array(bboxes)
#    draw_bbox(disp_bgr, bboxes)
#    imshow2(disp_bgr, B, figsize=(17, 17))

# %%
for I, L in zip(Is, Ls):
    B = get_prediction_bw(img_ppl, img_clf, I)
    B = gray_from_bw(B)   
    B = clean_bw(B)

    blobs, cntrs = region_props_bw(B, min_area=5)
    B = fill_bw(B, cntrs)
    
    disp_bgr = img_ppl.named_steps['remove_dark'].image.copy()
    v = img_ppl.named_features['hsv'].image[:,:,-1]
    bw = B
    bboxes = []
    areas = blobs['prop'][:, 0]
    area_thresh = np.mean(areas)
    for blob in blobs:
        bbox = blob['bbox']
        prop = blob['prop']
        v_bbox = extract_bbox(v, bbox, copy=True)
        bw_bbox = extract_bbox(bw, bbox)
        v_bbox[bw_bbox==0] = 0
        _, _, w, h = bbox
        min_dist = min(np.sqrt(w * h) / 5, 10)
        
        area, aspect, extent = blob['prop']
        if area > area_thresh and (aspect > 1.4 or extent < 0.5):
            image_max = ndi.maximum_filter(v_bbox, size=3, mode='constant')
            local_max = peak_local_max(image_max, min_distance=min_dist,
                                       indices=False, exclude_border=False)
            local_max = gray_from_bw(local_max)
            points = loca_max_point(local_max)
            disp_bgr_bbox = extract_bbox(disp_bgr, bbox)  
            if len(points) > 0:
                draw_point(disp_bgr_bbox, points)
            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
            imshow2(disp_bgr_bbox, image_max)
            print(min_dist, aspect, extent)
        else:
            bboxes.append(bbox)
    bboxes = np.array(bboxes)
    draw_bbox(disp_bgr, bboxes)
    imshow2(disp_bgr, B, figsize=(17, 17))
    