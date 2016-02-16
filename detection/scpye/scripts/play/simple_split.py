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
test_indices = [5]

# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

Is, Ls = drd.load_image_label_list(test_indices)

# %%
#for I, L in zip(Is, Ls):
#    B = get_prediction_bw(img_ppl, img_clf, I)
#    B = gray_from_bw(B)   
#    B = clean_bw(B)
#
#    blobs, cntrs = region_props_bw(B, min_area=5)
#    B = fill_bw(B, cntrs)
#    
#    disp_bgr = img_ppl.named_steps['remove_dark'].image.copy()
#    v = img_ppl.named_features['hsv'].image[:,:,-1]
#    
#    bw = B
#    bboxes = []
#    areas = blobs['prop'][:, 0]
#    area_thresh = np.mean(areas)
#    for blob in blobs:
#        bbox = blob['bbox']
#        prop = blob['prop']
#        v_bbox = extract_bbox(v, bbox, copy=True)
#        bw_bbox = extract_bbox(bw, bbox)
#        v_bbox[bw_bbox==0] = 0
#        _, _, w, h = bbox
#        min_dist = min(np.sqrt(w * h) / 5, 10)
#        
#        area, aspect, extent = blob['prop']
#        if area > area_thresh and (aspect > 1.4 or extent < 0.5):
#            image_max = ndi.maximum_filter(v_bbox, size=3, mode='constant')
#            local_max = peak_local_max(image_max, min_distance=min_dist,
#                                       indices=False, exclude_border=False)
#            local_max = gray_from_bw(local_max)
#            points = local_max_points(local_max)
#            disp_bgr_bbox = extract_bbox(disp_bgr, bbox)  
#            if len(points) > 0:
#                draw_point(disp_bgr_bbox, points)
#            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
#            imshow2(disp_bgr_bbox, image_max)
#            print(min_dist, aspect, extent)
#        else:
#            bboxes.append(bbox)
#    bboxes = np.array(bboxes)
#    draw_bbox(disp_bgr, bboxes)
#    imshow2(disp_bgr, B, figsize=(17, 17))

blb_anl = BlobAnalyzer(split=True)

for I, L in zip(Is, Ls):
    B = get_prediction_bw(img_ppl, img_clf, I)
    B = gray_from_bw(B)   
    B = clean_bw(B)

    blobs, cntrs = region_props_bw(B, min_area=5)
    B = fill_bw(B, cntrs)
    
    disp_bgr = img_ppl.named_steps['remove_dark'].image.copy()
    v = img_ppl.named_features['hsv'].image[:,:,-1]
    
    fruits = blb_anl.analyze(B, v)
    disp_bgr = img_ppl.named_steps['remove_dark'].image.copy()
    draw_bboxes(disp_bgr, fruits[:, :4])
    
imshow(disp_bgr, figsize=(17, 17))