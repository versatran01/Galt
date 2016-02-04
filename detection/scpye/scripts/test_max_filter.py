# -*- coding: utf-8 -*-
"""
Created on Thu Feb  4 14:19:48 2016

@author: makineni
"""
import numpy as np
from scpye.blob import BlobAnalyzer
from scpye.data_reader import DataReader
from scpye.train import load_data
from scpye.viz import imshow, imshow2
import cv2

from scipy import ndimage as ndi
import matplotlib.pyplot as plt
from skimage.feature import peak_local_max
from skimage import data, img_as_float

from scpye.image_pipeline import ImagePipeline
from scpye.image_transformer import MaximumFilterTransformer, DarkRemover

dr = DataReader('/home/makineni/Desktop/', color='green', mode='slow_flash')
image = dr.load_image(1)
imshow(image)

#b, g, r = cv2.split(image)
#imshow(r)
r = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


im = img_as_float(r)
# %%
# image_max is the dilation of im with a 20*20 structuring element
# It is used within peak_local_max function
s = 30
image_max = ndi.maximum_filter(im, size=s, mode='constant')

# Comparison between image_max and im to find the coordinates of local maxima
coordinates = peak_local_max(im, min_distance=s)

# %%
imshow(image_max)
ax = imshow(im)
ax.plot(coordinates[:, 1], coordinates[:, 0], 'r.')
ax.autoscale(tight=True)






