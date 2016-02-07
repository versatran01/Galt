# %%
import sys

# HACK
sys.path.append('..')

import os
import numpy as np
from sklearn.externals import joblib
from scpye.visualization import imshow, imshow2
from scpye.data_reader import DataReader
from scpye.train import *

# %%
base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
train_inds = range(0, 12, 3)
test_inds = range(1, 12, 3)
save = True
load = False

# Parameters
k = 0.4
v_min = 25
if color == 'red':
    bbox = np.array([200, 0, 800, 1440])
    use_loc = False
else:
    bbox = np.array([200, 0, 800, 1440])
    use_loc = True
    
drd = DataReader(base_dir=base_dir, color=color, mode=mode)
img_ppl_pkl = os.path.join(drd.model_dir, 'img_ppl.pkl')
img_clf_pkl = os.path.join(drd.model_dir, 'img_clf.pkl')

# %%
# DataReader
if not load:
    # ImagePipeline
    img_ppl = make_image_pipeline(bbox=bbox, k=k, v_min=v_min, use_loc=use_loc)
    # Train SVC
    img_clf = train_image_classifier(drd, train_inds, img_ppl)

    if save:
        print('Svaing pipeline and classifier')
        joblib.dump(img_ppl, img_ppl_pkl)
        joblib.dump(img_clf, img_clf_pkl)
else:
    img_ppl = joblib.load(img_ppl_pkl)
    img_clf = joblib.load(img_clf_pkl)
    
# %%
I, L = drd.load_image_label(1)
img_ppl.transform(I, L)

# Get transformed label
lbl = img_ppl.named_steps['remove_dark'].label
pos = lbl[:,:, 1]

# %%
X = img_ppl.transform(I)
y = img_clf.predict(X)
bw = img_ppl.named_steps['remove_dark'].mask.copy()
bw[bw > 0] = y
bw = np.array(bw, dtype='uint8') * 255

bgr = img_ppl.named_steps['remove_dark'].img
imshow2(bgr, bw)
