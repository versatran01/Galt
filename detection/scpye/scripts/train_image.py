# %%
import numpy as np
from scpye.data_reader import DataReader
from scpye.training import make_image_pipeline, train_image_classifier
from scpye.testing import test_image_classifier

# %%
base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
side = 'north'

# %%
train = True
save = True
test = True

# %% 
# Parameters
k = 0.5
v_min = 28

if side == 'north':
    train_inds = range(0, 12, 3) + range(1, 12, 3) + range(2, 12, 3)
    test_inds = range(2, 12, 3)
else:
    train_inds = range(12, 16)
    test_inds = range(12, 16)

if color == 'red':
    bbox = np.array([350, 0, 500, 1440])
    use_loc = True
    method = 'lr'
    cspace = ['hsv', 'lab']
else:
    bbox = np.array([350, 240, 500, 1440])
    use_loc = True
    method = 'svm'
    cspace = ['hsv', 'lab']

# %%
# DataReader
drd = DataReader(base_dir, color=color, mode=mode, side=side)
if train:
    img_ppl = make_image_pipeline(bbox=bbox, k=k, v_min=v_min, cspace=cspace,
                                  use_loc=use_loc)
    img_clf = train_image_classifier(drd, train_inds, img_ppl, method=method)

    if save:
        print('Saving pipeline and classifier')
        drd.save_model(img_ppl, 'img_ppl')
        drd.save_model(img_clf, 'img_clf')
        
# %%
if test:
    img_ppl = drd.load_model('img_ppl')
    img_clf = drd.load_model('img_clf')

    test_image_classifier(drd, test_inds, img_ppl, img_clf)