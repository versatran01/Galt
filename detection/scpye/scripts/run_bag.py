from scpye.data_reader import DataReader
from scpye.fruit_detector import FruitDetector
from scpye.blob_analyzer import BlobAnalyzer
from scpye.fruit_tracker import FruitTracker
from scpye.fruit_visualizer import FruitVisualizer

base_dir = '/home/chao/Workspace/bag'
color = 'red'
mode = 'slow_flash'
bag_ind = 3

dr = DataReader(base_dir, color=color, mode=mode)
fd = FruitDetector.from_pickle(dr.model_dir)
ba = BlobAnalyzer(split=False, min_area=5)
ft = FruitTracker()
fv = FruitVisualizer(image_dir=dr.image_dir)

for image in dr.load_bag(bag_ind):
    bw = fd.detect(image)
    fruits, bw_clean = ba.analyze(bw, fd.v)
    ft.track(fd.color, fruits)
    fv.show(ft.disp, bw_clean)
    print(ft.total_counts)

ft.finish()
print(ft.total_counts)