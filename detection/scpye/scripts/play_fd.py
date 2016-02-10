# %%
from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.visualization import *

# %%
base_dir = '/home/chao/Dropbox'
color = 'red'
mode = 'slow_flash'
test_indices = [5, 6, 7]

# %%
drd = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(drd.model_dir)

Is, Ls = drd.load_image_label_list(test_indices)

for I in Is:
    fruits, bw = fd.detect(I)
    disp = fd.color
    draw_bboxes(disp, fruits[:, :4])
    imshow(disp, figsize=(17, 17))