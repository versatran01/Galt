# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 23:17:29 2016

@author: chao
"""

# %%
import numpy as np
from scpye.data_reader import DataReader
from scpye.testing import get_positive_bw, get_prediction_bw
from scpye.visualization import imshow2, imshow
from scpye.blob_analysis import clean_bw, gray_from_bw

# %%
base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'

# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

I, L = drd.load_image_label(1)
bw_pos = get_positive_bw(img_ppl, I, L)
bw_clf = get_prediction_bw(img_ppl, img_clf, I)
bw_pos = gray_from_bw(bw_pos)
bw_clf = gray_from_bw(bw_clf)

# clean bw
bw_clf = clean_bw(bw_clf)
bw_tp = bw_pos & bw_clf
imshow(bw_tp)
imshow2(bw_pos, bw_clf)


# Do blob analysis on bw_clf
# For each blob/cntr, use bbox to extract region in bw_tp
# Find how many contours in the region and classify to [0, 1, n]
# Feature is blobs [w, h, area, extent, solid, eccen]
# Then apply it to new detection and classify to [0, 1, n]
# Do a water shed on [n] case and split bbox