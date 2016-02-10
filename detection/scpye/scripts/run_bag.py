from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.fruit_tracker import FruitTracker
from scpye.fruit_visualizer import FruitVisualizer
from scpye.visualization import imshow

base_dir = '/home/chao/Workspace/bag'
color = 'green'
mode = 'slow_flash'

dr = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(dr.model_dir, split=False)
ft = FruitTracker()
fv = FruitVisualizer()

i = 0
for image in dr.load_bag(3):
    fruits = fd.detect(image)
    ft.track(fd.color, fruits)
    imshow(ft.disp)
    i += 1
    if i == 2:
        break
