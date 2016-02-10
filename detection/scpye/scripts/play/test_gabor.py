# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 12:16:54 2016

@author: anuragmakineni
"""
#%%
import numpy as np
from scpye.data_reader import DataReader
from scpye.visualization import imshow, imshow2
import cv2

from scpye.image_pipeline import ImagePipeline
import matplotlib.pyplot as plt

# %%

dr = DataReader('/home/chao/Dropbox/', color='green', mode='slow_flash')
image = dr.load_image(5)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.pyrDown(gray)
gray = np.rot90(gray, -1)

#%%
frequency = 1.0
theta = 0
theta = (theta / 4.0) * np.pi

#%%
real_0, imag_0 = gabor_filter(gray, frequency, theta)
theta = 45
theta = (theta / 4.0) * np.pi

imshow(real_0)

real_45, imag_45 = gabor_filter(gray, frequency, theta)
theta = 90
theta = (theta / 4.0) * np.pi

imshow(real_45)

real_90, imag_90 = gabor_filter(gray, frequency, theta)
theta = 135
theta = (theta / 4.0) * np.pi

imshow(real_90)


real_135, imag_135 = gabor_filter(gray, frequency, theta)

imshow(real_135)


filtered = real_0 + real_45 + real_90 + real_135

plt.figure(figsize=(16, 16))
plt.imshow(filtered, cmap="jet")

plt.figure(figsize=(16, 16))
plt.imshow(gray, cmap="jet")


# %%
import cv2

ksize = 5
sigma = 1
theta = np.pi
lambd = 5.0
gamma = 0.5
psi = 0
kernel = cv2.getGaborKernel((ksize, ksize), sigma, theta, lambd, gamma, psi)
kernel /= kernel.sum()

fimg = cv2.filter2D(gray, cv2.CV_8UC3, kernel)
imshow2(gray, fimg)
