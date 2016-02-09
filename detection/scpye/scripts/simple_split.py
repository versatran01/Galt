# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 20:45:49 2016

@author: chao
"""

# %%
import numpy as np
from scpye.data_reader import DataReader
from scpye.testing import get_positive_bw, get_prediction_bw
from scpye.visualization import *
from scpye.blob_analysis import *
from scpye.bounding_box import extract_bbox
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from scpye.training import *
from sklearn.grid_search import GridSearchCV
from skimage.measure import label
from skimage.feature import peak_local_max
import scipy.ndimage as ndi
from skimage.morphology import watershed

# %%
base_dir = '/home/chao/Dropbox'
color = 'green'
mode = 'slow_flash'
test_indices = [5]

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
        bw_bbox = extract_bbox(B, bbox)
        v_bbox = extract_bbox(v, bbox, copy=True)
        v_bbox[bw_bbox == 0] = 0
        min_dist = np.sqrt(bbox[-1] * bbox[-2]) / 5
        rc = peak_local_max(v_bbox, min_distance=min_dist)
        if len(rc) > 1:
            # Do watershed to split
            # Seems like v_bbox is better than distance transform
#            dist = ndi.distance_transform_edt(bw_bbox)
#            dist += (v_bbox / v_bbox.max() * dist.max()) 
            dist = v_bbox
            local_max = peak_local_max(dist, indices=False,
                                       min_distance=min_dist,
                                       labels=bw_bbox)
            markers = ndi.label(local_max, structure=np.ones((3, 3)))[0]
            labels = watershed(-dist, markers, mask=bw_bbox)
            # Given labels
            # Generate new bounding boxes
            # Should be easy
            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
            draw_point(extract_bbox(disp_bgr, bbox), np.fliplr(rc))
            imshow3(extract_bbox(disp_bgr, bbox), dist, labels)
        else:
            draw_bbox(disp_bgr, bbox, color=(0, 0, 255))
        draw_bbox(disp_bgr, blobs[:-10]['bbox'])
    imshow2(disp_bgr, B, figsize=(17, 17))