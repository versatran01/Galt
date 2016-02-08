# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 23:17:29 2016

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

# %%
base_dir = '/home/chao/Dropbox'
color = 'green'
mode = 'slow_flash'
train_indices = range(0, 12, 3) + range(2, 12, 3)
test_indices = range(1, 12, 3)

# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

Is, Ls = drd.load_image_label_list(train_indices)

# TODO:
# fd = FruitDetector.from_pickle(drd.model_dir)
# bw_pos = fd.get_positive(I, L)
# bw_clf = fd.predict(I)
Xs = []
ys = []
for I, L in zip(Is, Ls):
    bw_pos = get_positive_bw(img_ppl, I, L)
    bw_clf = get_prediction_bw(img_ppl, img_clf, I)

    bw_pos = gray_from_bw(bw_pos)   
    bw_clf = gray_from_bw(bw_clf)

    bw_clf = clean_bw(bw_clf)

    blobs, cntrs = region_props_bw(bw_clf)
    bw_clf = fill_bw(bw_clf, cntrs)

    bw_tp = bw_clf & bw_pos

    y = []
    for blob in blobs:
        bbox = blob['bbox']
        bw_clf_bbox = extract_bbox(bw_clf, bbox)
        bw_pos_bbox = extract_bbox(bw_pos, bbox)
    
        l, n = label(bw_pos_bbox, return_num=True)
        area_pos = np.count_nonzero(l)
        area_clf = blob['prop'][0]
        if n == 1 or area_clf / area_pos > 10:
            # Not apple
            ys.append(0)
        elif n == 2 :
            # Single apple
            ys.append(1)
        else:
            # Multiple apple
            ys.append(2)
    Xs.append(blobs['prop'])
X = np.vstack(Xs)
y = np.array(ys, np.float)

scaler = StandardScaler()
Xt = scaler.fit_transform(X)
svc = SVC()
param_grid = [{'C': [1, 10, 100, 500, 1000]}]
grid = GridSearchCV(estimator=svc, param_grid=param_grid, cv=5, verbose=5)
grid.fit(Xt, y)
print('Finish training')

# %%
# Test on a new image
Is, Ls = drd.load_image_label_list(test_indices)

for I, L in zip(Is, Ls):
    bw_pos = get_positive_bw(img_ppl, I, L)
    bw_clf = get_prediction_bw(img_ppl, img_clf, I)
    bw_pos = gray_from_bw(bw_pos)
    bw_clf = gray_from_bw(bw_clf)

    bw_clf = clean_bw(bw_clf)
    blobs, cntrs = region_props_bw(bw_clf)
    bw_clf = fill_bw(bw_clf, cntrs)

    bgr = img_ppl.named_steps['remove_dark'].image
    disp_bgr = bgr.copy()

    X = blobs['prop']
    Xt = scaler.transform(X)
    y_clf = grid.predict(Xt)

    for blob, r in zip(blobs, y_clf):
        bbox = blob['bbox']
        if r == 0:
            draw_bbox(disp_bgr, bbox)
        elif r == 1:
            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
        else:
            draw_bbox(disp_bgr, bbox, color=(0, 0, 255))

    imshow2(disp_bgr, bw_clf, figsize=(17, 17))
        