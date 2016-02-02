# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 13:19:54 2016

@author: chao
"""

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
from skimage.feature import hog
from skimage import exposure

# %%

def imshow(image, title="", figsize=None):
    plt.figure(figsize=figsize).gca().imshow(image)
    plt.gca().set_title(title)

# %%
data_dir = "/home/chao/Workspace/bag/apple/green/fast_led/train"
i = 0
img_fmt = "frame{0:04d}_{1}.png"

img_file = os.path.join(data_dir, img_fmt.format(i, 'raw'))
neg_file = os.path.join(data_dir, img_fmt.format(i, 'neg'))
pos_file = os.path.join(data_dir, img_fmt.format(i, 'pos'))

img_raw = cv2.imread(img_file, cv2.IMREAD_COLOR)

img_gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
img_gray = cv2.pyrDown(img_gray)
fd, img_hog = hog(img_gray, orientations=8, pixels_per_cell=(8, 8),
                  cells_per_block=(1, 1), visualise=True)
imshow(img_hog, figsize=(16,16))
