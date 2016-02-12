# -*- coding: utf-8 -*-
"""
Created on Thu Feb 11 21:34:34 2016

@author: chao
"""

# %% 
import cv2
import numpy as np
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.blob_analyzer import BlobAnalyzer
from scpye.visualization import imshow, imshow2

# %%
dr = DataReader()
fd = FruitDetector.from_pickle(dr.model_dir)
I = dr.load_image(1)
bw = fd.detect(I)

# %%
ba = BlobAnalyzer(min_area=5, split=False)
fruits, bw_clean = ba.analyze(bw, fd.v)
