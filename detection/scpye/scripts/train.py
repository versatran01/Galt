# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 16:06:13 2016

@author: chao
"""

# %%
import os
import numpy as np
import matplotlib.pyplot as plt
import cv2
from scpye.image_transformer import *
from scpye.image_pipeline import ImagePipeline, FeatureUnion
from sklearn import svm


# %%


def imshow(image, title=""):
    plt.figure(figsize=(10, 10)).gca().imshow(image)
    plt.gca().set_title(title)


def extract_bbox(image, bbox):
    x, y, w, h = bbox
    return image[y:(y + h), x:(x + w), ...]


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
# When training
#   pipeline.fit(X, y)
# When testing
#   pipeline.predict(X)

# X = img_raw
# y = np.dstack((neg, pos))

features = FeatureUnion([
   ('bgr', CspaceTransformer('bgr')),
   ('hsv', CspaceTransformer('hsv')),
   ('mask_location', MaskLocator())
])


Xs = [img_raw0, img_raw1]
ys = [lbl0, lbl1]

#Xs = img_raw0
#ys = lbl0

image_ppl = ImagePipeline([
    ('rotate_image', ImageRotator(-1)),
    ('crop_image', ImageCropper(bbox)),
    ('resize_image', ImageResizer()),
    ('remove_dark', DarkRemover(25)),
    ('features', features),
    ('scaler', StandardScaler()),
    ('svc', svm.SVC())
])

image_ppl.fit(Xs, ys)

# %%
img_file = os.path.join(data_dir, img_fmt.format(2, 'raw'))
img_raw2 = cv2.imread(img_file, cv2.IMREAD_COLOR)
y_pred = image_ppl.predict(img_raw2)
bw = image_ppl.named_steps['remove_dark'].mask
bw[bw > 0] = y_pred
imshow(bw)
