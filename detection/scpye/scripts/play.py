# -*- coding: utf-8 -*-
"""
Created on Sun Jan 31 16:06:13 2016

@author: chao
"""

# %%
import os
import cv2
import numpy as np
from scpye.viz import imshow
from scpye.data_reader import DataReader
from scpye.train import make_image_pipeline, SVC

# %%
dr = DataReader()
X, y = dr.load_image_label(0)
imshow(X)
ppl = make_image_pipeline()
Xt, yt = ppl.fit_transform(X, y)
clf = SVC()
