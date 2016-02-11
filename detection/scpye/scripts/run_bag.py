from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.fruit_tracker import FruitTracker
from scpye.fruit_visualizer import FruitVisualizer

base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
bag_ind = 1

dr = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(dr.model_dir, split=False)
ft = FruitTracker()
fv = FruitVisualizer(image_dir=dr.image_dir)

for image in dr.load_bag(bag_ind):
    fruits = fd.detect(image)
    ft.track(fd.color, fruits)
    fv.show(ft.disp, fd.bw)
    print(ft.total_counts)

ft.finish()
print(ft.total_counts)
