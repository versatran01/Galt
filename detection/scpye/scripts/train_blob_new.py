# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 19:20:09 2016

@author: chao
"""

import numpy as np
from scpye.image_pipeline import ImagePipeline
from scpye.image_transformer import ImageTransformer, FeatureTransformer
from scpye.blob_analyzer import clean_bw, gray_from_bw, fill_bw
from scpye.region_props import region_props_bw, clean_bw, fill_bw, gray_from_bw
from scpye.bounding_box import extract_bbox
from skimage.measure import label
from scpye.data_reader import DataReader
from scpye.testing import get_positive_bw, get_prediction_bw
from sklearn.preprocessing import StandardScaler
from sklearn.grid_search import GridSearchCV
from sklearn.svm import SVC
from scpye.visualization import *

# %%

class BinaryDenoiser(ImageTransformer):
    def __init__(self, ksize=3, iters=1):
        self.iters = iters
        self.ksize = ksize
        self.bw = None
    
    @ImageTransformer.forward_list_input
    def transform(self, X, y=None):
        # assume X is grayscale
        Xt = clean_bw(X, ksize=self.ksize, iters=self.iters)
        self.bw = Xt
        
        if y is None:
            return Xt
        else:
            # y doesn't need to be cleaned
            return Xt, y

class BlobFinder(ImageTransformer):
    def __init__(self, min_area=8, max_area_ratio=8):
        self.min_area = min_area
        self.max_area_ratio = max_area_ratio
        self.bw = None
        self.blobs = None
        self.cntrs = None
        
    @ImageTransformer.forward_list_input
    def transform(self, X, y=None):
        blobs, cntrs = region_props_bw(X, min_area=self.min_area)
        self.blobs = blobs
        self.cntrs = cntrs
        self.bw = fill_bw(X, cntrs)
        
        Xt = blobs['prop']
        if y is None:
            return Xt
        else:
            yt = []
            tp = X & y
            for blob in blobs:
                prop = blob['prop']
                bbox = blob['bbox']
                y_bbox = extract_bbox(tp, bbox)
    
                l, n = label(y_bbox, return_num=True)
                n -= 1 # remove background label
                area_y = np.count_nonzero(l)
                area_X = prop[0]
                
                if area_X < self.min_area * self.max_area_ratio:
                    # Small blobs will always be single apple
                    yt.append(1)
                else:
                    # For bigger blob, we look at whether there is a label in
                    # the bounding box
                    if n == 0:
                        # Not an apple when no label found
                        yt.append(0)
                    else:
                        if (area_X / area_y) > self.max_area_ratio:
                            # Found a label but detection is too big 
                            # Not an apple
                            yt.append(0)
                        elif n == 1:
                            # Found a label and detection and label are of
                            # Similar size
                            yt.append(1)
                        else:
                            # Found multiple labels
                            yt.append(2)
            yt = np.array(yt)
            return Xt, yt
            
class StackTransformer(FeatureTransformer):
    @FeatureTransformer.stack_list_input
    def transform(self, X, y=None):
        return X

bw_ppl = ImagePipeline([
    ('denoise_binary', BinaryDenoiser()),
    ('find_blob', BlobFinder()),
    ('stack', StackTransformer()),
    ('scale', StandardScaler())
])
        
# %%
base_dir = '/home/chao/Dropbox'
color = 'green'
mode = 'slow_flash'
train_indices = range(0, 12, 3) + range(1, 12, 3)
test_indices = range(2, 12, 3)

# %%
drd = DataReader(base_dir, color=color, mode=mode)
img_ppl = drd.load_model('img_ppl')
img_clf = drd.load_model('img_clf')

Is, Ls = drd.load_image_label_list(train_indices)

Bs = []
Ps = []
for I, L in zip(Is, Ls):    
    B = get_prediction_bw(img_ppl, img_clf, I)
    P = get_positive_bw(img_ppl, I, L)
    
    # True positive mask
    P = B & P
    
    B = gray_from_bw(B)
    P = gray_from_bw(P)
    
    Bs.append(B)
    Ps.append(P)

X_train, y_train = bw_ppl.fit_transform(Bs, Ps)
param_grid = [{'C': [1, 10, 100, 500, 1000]}]
bw_clf = GridSearchCV(estimator=SVC(), param_grid=param_grid, cv=4, verbose=5)
bw_clf.fit(X_train, y_train)
print('Finish training')

# %%
Is, Ls = drd.load_image_label_list(test_indices)

for I, L in zip(Is, Ls):
    B = get_prediction_bw(img_ppl, img_clf, I)
    disp_bgr = img_ppl.named_steps['remove_dark'].image.copy()
    
    B = gray_from_bw(B)
    X = bw_ppl.transform(B)
    y = bw_clf.predict(X)
    blobs = bw_ppl.named_steps['find_blob'].blobs
    disp_bw = bw_ppl.named_steps['find_blob'].bw
    
    for blob, r in zip(blobs, y):
        bbox = blob['bbox']
        if r == 0:
            draw_bbox(disp_bgr, bbox)
        elif r == 1:
            draw_bbox(disp_bgr, bbox, color=(0, 255, 0))
        else:
            draw_bbox(disp_bgr, bbox, color=(0, 0, 255))
    
    imshow2(disp_bgr, disp_bw, figsize=(17, 17))
        