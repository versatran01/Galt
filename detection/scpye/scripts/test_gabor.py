# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 12:16:54 2016

@author: anuragmakineni
"""
#%%
import numpy as np
from scpye.data_reader import DataReader
from scpye.viz import imshow, imshow2
import cv2

from skimage.filters import gabor_kernel, gabor_filter
from scpye.image_pipeline import ImagePipeline
# %%

dr = DataReader('/home/anuragmakineni/Desktop/', color='green', mode='slow_flash')
image = dr.load_image(2)
imshow(image)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#%%
frequency = 0.1
theta = 45

theta = (theta / 4.0) * np.pi
kernel = np.real(gabor_kernel(frequency, theta))
imshow(kernel)
#%%
real, imag = gabor_filter(gray, frequency, theta)
imshow(real)
#%%




