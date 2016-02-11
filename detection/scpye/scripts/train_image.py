# %%
import numpy as np
from scpye.data_reader import DataReader
from scpye.training import make_image_pipeline, train_image_classifier
from scpye.testing import test_image_classifier

# %%
base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
train_inds = range(0, 12, 3) + range(12, 16)
test_inds = range(1, 12, 3)

# %%
train = True
save = False
test = True

# %% 
# Parameters
k = 0.5
v_min = 30
if color == 'red':
    bbox = np.array([300, 0, 600, 1440])
    use_loc = False
else:
    bbox = np.array([250, 250, 700, 1440])
    use_loc = True

# %%
# DataReader
drd = DataReader(base_dir, color=color, mode=mode)
if train:
    img_ppl = make_image_pipeline(bbox=bbox, k=k, v_min=v_min, use_loc=use_loc)
    img_clf = train_image_classifier(drd, train_inds, img_ppl, method='lr')

    if save:
        print('Saving pipeline and classifier')
        drd.save_model(img_ppl, 'img_ppl')
        drd.save_model(img_clf, 'img_clf')
        
# %%
if test:
    img_ppl = drd.load_model('img_ppl')
    img_clf = drd.load_model('img_clf')

    test_image_classifier(drd, test_inds, img_ppl, img_clf)