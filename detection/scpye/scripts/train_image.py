# %%
import numpy as np
from scpye.data_reader import DataReader
from scpye.training import make_image_pipeline, train_image_classifier
from scpye.testing import test_image_classifier

# %%
base_dir = '/home/chao/Dropbox'
color = 'green'
mode = 'slow_flash'
train_inds = range(0, 12, 3)
test_inds = range(2, 12, 3)

# %%
train = True
save = True
test = True

# %% 
# Parameters
k = 0.4
v_min = 25
if color == 'red':
    use_loc = False
    bbox = np.array([240, 0, 720, 1440])
else:
    bbox = np.array([240, 250, 720, 1440])
    use_loc = True

# %%
# DataReader
drd = DataReader(base_dir=base_dir, color=color, mode=mode)
if train:
    img_ppl = make_image_pipeline(bbox=bbox, k=k, v_min=v_min, use_loc=use_loc)
    img_clf = train_image_classifier(drd, train_inds, img_ppl)

    if save:
        print('Saving pipeline and classifier')
        drd.save_model(img_ppl, 'img_ppl')
        drd.save_model(img_clf, 'img_clf')

if test:
    img_ppl = drd.load_model('img_ppl')
    img_clf = drd.load_model('img_clf')

    test_image_classifier(drd, test_inds, img_ppl, img_clf)