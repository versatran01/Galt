# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 16:06:13 2016

@author: chao
"""

# %%
import os
import cv2
import numpy as np
from scpye.viz import imshow, imshow2
from scpye.data_reader import DataReader
from scpye.train import make_image_pipeline, load_data, transform_data, train_svm

# %%
# bbox = np.array([200, 0, 800, 1440])
bbox = np.array([200, 0, 800, 1440])

dr = DataReader(color='red', mode='fast_flash')
ppl = make_image_pipeline(bbox=bbox, k=0.4, v_min=25, use_loc=False)

train_inds = range(0, 12, 3)
Is, Ls = load_data(dr, train_inds)

X_train, y_train = transform_data(ppl, Is, Ls)
clf = train_svm(X_train, y_train)

# %%
img, lbl = dr.load_image_label(1)
X = ppl.transform(img)
y = clf.predict(X)
bw = ppl.named_steps['remove_dark'].mask.copy()
bw[bw > 0] = y
bgr = ppl.named_steps['features'].transformer_list[0][-1].img
imshow2(bgr, bw, fsize=(16, 16))
