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
from scpye.train import *
from sklearn.cross_validation import train_test_split

# %%
test_size = 0.3
bbox = np.array([200, 0, 800, 1440])
params = [{'C': [0.1, 1, 10]}]

dr = DataReader()
ppl = make_image_pipeline(bbox=bbox)

# Load a list of training images and labels
train_indices = range(0, 6, 2)
imgs_train = []
lbls_train = []
for ind in train_indices:
    img, lbl = dr.load_image_label(ind)
    imgs_train.append(img)
    lbls_train.append(lbl)

# Transform both images and labels
print('Transforming images...')
X_train, y_train = ppl.fit_transform(imgs_train, lbls_train)
print('Done')

# Split data into train and valid
print('Spliting training data...')
X_t, X_v, y_t, y_v = train_test_split(X_train, y_train, test_size=test_size)
print('Done')

# Do a grid search over some parameters
print('Training classifier...')
clf = tune_svc(X_t, y_t, params)
print('Done')

# %%
print_grid_search_report(clf)
print_validation_report(clf, X_v, y_v)

# %%
img, lbl = dr.load_image_label(9)
X = ppl.transform(img)
y = clf.predict(X)
bw = ppl.named_steps['remove_dark'].mask.copy()
bw[bw > 0] = y
bgr = ppl.named_steps['features'].transformer_list[0][-1].img
imshow2(bgr, bw)

