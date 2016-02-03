# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 16:06:13 2016

@author: chao
"""

# %%
import os
import cv2
import numpy as np
from sklearn.grid_search import GridSearchCV
from scpye.viz import imshow
from sklearn.svm import SVC

# %%
data_dir = "/home/chao/Workspace/bag/apple/green/fast_led/train"
i = 0
img_fmt = "frame{0:04d}_{1}.png"

img_file = os.path.join(data_dir, img_fmt.format(i, 'raw'))
neg_file = os.path.join(data_dir, img_fmt.format(i, 'neg'))
pos_file = os.path.join(data_dir, img_fmt.format(i, 'pos'))

img_raw0 = cv2.imread(img_file, cv2.IMREAD_COLOR)
neg0 = cv2.imread(neg_file, cv2.IMREAD_GRAYSCALE)
pos0 = cv2.imread(pos_file, cv2.IMREAD_GRAYSCALE)
lbl0 = np.dstack((neg0, pos0))

i = 1
img_file = os.path.join(data_dir, img_fmt.format(i, 'raw'))
neg_file = os.path.join(data_dir, img_fmt.format(i, 'neg'))
pos_file = os.path.join(data_dir, img_fmt.format(i, 'pos'))

img_raw1 = cv2.imread(img_file, cv2.IMREAD_COLOR)
neg1 = cv2.imread(neg_file, cv2.IMREAD_GRAYSCALE)
pos1 = cv2.imread(pos_file, cv2.IMREAD_GRAYSCALE)
lbl1 = np.dstack((neg1, pos1))

bbox = np.array([200, 200, 800, 1400])

# %%
# Use Pipeline and FeatureUnion to simplify preprocessing
Xs = [img_raw0, img_raw1]
ys = [lbl0, lbl1]

# Xs = img_raw0
# ys = lbl0
params = dict(C=[0.1, 1, 10])

ppl = make_image_pipeline(bbox=bbox, use_loc=True)
Xts, yts = ppl.fit_transform(Xs, ys)

clf = GridSearchCV(SVC(), param_grid=params, verbose=10)
clf.fit(Xts, yts)

# %%
img_file = os.path.join(data_dir, img_fmt.format(2, 'raw'))
img_raw2 = cv2.imread(img_file, cv2.IMREAD_COLOR)
y_pred = clf.predict(ppl.transform(img_raw2))
bw = ppl.named_steps['remove_dark'].mask
bw[bw > 0] = y_pred
imshow(bw)
