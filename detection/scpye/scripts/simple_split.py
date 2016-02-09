# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 20:45:49 2016

@author: chao
"""

# %%
from scpye.blob_analysis import *
from scpye.bounding_box import extract_bbox
from scpye.region_props import clean_bw, fill_bw, gray_from_bw
from scpye.testing import get_prediction_bw
from scpye.training import *
from scpye.visualization import *

# %%
base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'
test_indices = [3]

# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

Is, Ls = drd.load_image_label_list(test_indices)

for I, L in zip(Is, Ls):
    B = get_prediction_bw(img_ppl, img_clf, I)
    B = gray_from_bw(B)   
    B = clean_bw(B)

    blobs, cntrs = region_props_bw(B, min_area=9)
    B = fill_bw(B, cntrs)
    
    disp_bgr = img_ppl.named_steps['remove_dark'].image
    v = img_ppl.named_features['hsv'].image[:,:,-1]
    
    blobs = np.partition(blobs, len(blobs) - 10, order='area')
    for blob in blobs[-10:]:
        bbox = blob['bbox']
        label, n = label_blob(bbox, B, v, return_num=True)
        if n == 1:
            draw_bbox(disp_bgr, bbox, color=(0, 0, 255))
        else:
            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
            imshow2(extract_bbox(disp_bgr, bbox), label)
        draw_bbox(disp_bgr, blobs[:-10]['bbox'])
    imshow2(disp_bgr, B, figsize=(17, 17))
    