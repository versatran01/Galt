# %%
import sys

# HACK
sys.path.append('..')

import numpy as np
from scpye.viz import imshow, imshow2
from scpye.data_reader import DataReader
from scpye.train import (make_image_pipeline, load_data, fit_transform_data,
                         train_svm)


# %%
def train_image_classifier(drd, inds, ppl):
    Is, Ls = load_data(drd, inds)
    X_train, y_train = fit_transform_data(ppl, Is, Ls)
    clf = train_svm(X_train, y_train)
    return clf


def test_data(drd, inds, ppl, clf):
    for ind in inds:
        pass


# %%
base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'
train_inds = range(0, 12, 3)
test_inds = range(1, 12, 3)
save = True

# Parameters
k = 0.4
v_min = 25
if color == 'red':
    bbox = np.array([200, 0, 800, 1440])
    use_loc = False
else:
    bbox = np.array([200, 0, 800, 1440])
    use_loc = True

# %%
# DataReader
drd = DataReader(base_dir=base_dir, color=color, mode=mode)
# ImagePipeline
img_ppl = make_image_pipeline(bbox=bbox, k=k, v_min=v_min, use_loc=use_loc)
# Train SVC
img_clf = train_image_classifier(drd, train_inds, img_ppl)

# %%
I, L = drd.load_image_label(1)
img_ppl.transform(I, L)

# Get transformed label
lbl = img_ppl.named_steps['remove_dark'].label
pos = lbl[:,:, 1]
imshow(pos)

# %%
X = img_ppl.transform(I)
y = img_clf.predict(X)
bw = img_ppl.named_steps['remove_dark'].mask.copy()
bw[bw > 0] = y
bw = np.array(bw, dtype='uint8') * 255

bgr = img_ppl.named_steps['features'].transformer_list[0][-1].img
imshow2(bgr, bw, fsize=(16, 16))
