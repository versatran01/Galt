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
from sklearn.pipeline import Pipeline
from scpye.image_pipeline import ImagePipeline


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
pos_file = os.path.join(data_dir, img_fmt.format(i, 'pos'))
neg_file = os.path.join(data_dir, img_fmt.format(i, 'neg'))

img_raw = cv2.imread(img_file, cv2.IMREAD_COLOR)
pos = cv2.imread(pos_file, cv2.IMREAD_GRAYSCALE)
neg = cv2.imread(neg_file, cv2.IMREAD_GRAYSCALE)

# %%
# image transformation

# rotate image
img_rot = np.rot90(img_raw, -1)
imshow(img_rot, 'rot')

# crop image
bbox = [200, 210, 800, 1500]
img_crop = extract_bbox(img_rot, bbox)
imshow(img_crop, 'crop')

# resize image (either resize or gaussian pyramid)
k = 0.5
# img_down = cv2.resize(im_crop, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)
img_pyr = cv2.pyrDown(img_crop)
imshow(img_pyr, 'pyr')

# %%
# convert to feature
img_bgr = img_pyr

# convert to hsv
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
imshow(img_hsv, 'hsv')

# convert to lab
img_lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB)
imshow(img_lab, 'lab')

# mask out pixels that have low value
v = img_hsv[:, :, -1]
mask = v > 25
imshow(mask, 'mask')

# %%
# Use Pipeline and FeatureUnion to simplify preprocessing
# When training
#   pipeline.fit(X, y)
# When testing
#   pipeline.predict(X)

X = img_raw
y = np.dstack((neg, pos))

image_ppl = ImagePipeline([
    ('rotate_image', ImageRotator(-1)),
    ('crop_image', ImageCropper(bbox)),
    ('resize_image', ImageResizer())
])


Xt = image_ppl.transform(X)

# %%
