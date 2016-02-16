from __future__ import (print_function, absolute_import, division)

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt


class FruitVisualizer(object):
    def __init__(self, pause_time=0.001, image_dir=None):
        self.fig = plt.figure()
        plt.ion()
        self.ax_bgr = self.fig.add_subplot(121)
        self.ax_bw = self.fig.add_subplot(122)
        self.h_bgr = None
        self.h_bw = None

        self.pause_time = pause_time
        self.margin = 40
        self.i = 0

        self.image_dir = image_dir
        self.image_name = 'image{0:04d}.png'

    def show(self, disp_bgr, disp_bw):
        if self.h_bgr is None:
            self.h_bgr = self.ax_bgr.imshow(disp_bgr)
            self.h_bw = self.ax_bw.imshow(disp_bw)
        else:
            self.h_bgr.set_data(disp_bgr)
            self.h_bw.set_data(disp_bw)

        self.i += 1

        if self.image_dir is not None:
            # disp_bw = cv2.cvtColor(disp_bw, cv2.COLOR_GRAY2BGR)
            # white = np.ones((len(disp_bw), self.margin, 3)) * 255
            # disp = np.hstack((disp_bgr, white, disp_bw))
            image_name = os.path.join(self.image_dir,
                                      self.image_name.format(self.i))
            cv2.imwrite(image_name, disp_bgr)

        plt.pause(self.pause_time)
