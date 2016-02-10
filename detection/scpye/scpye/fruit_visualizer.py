from __future__ import (print_function, absolute_import, division)

import cv2
import matplotlib.pyplot as plt


class FruitVisualizer(object):
    def __init__(self, t=0.001, save=False):
        self.fig = plt.figure()
        plt.ion()
        self.ax_bgr = self.fig.add_subplot(121)
        self.ax_bw = self.fig.add_subplot(122)
        self.h_bgr = None
        self.h_bw = None
        self.init = False
        self.t = t
        self.save = save

    def show(self, disp_bgr, disp_bw):
        if not self.init:
            self.h_bgr = self.ax_bgr.imshow(disp_bgr)
            self.h_bw = self.ax_bw.imshow(disp_bw, cmap=plt.cm.gray)
        else:
            self.h_bgr.set_data(disp_bgr)
            self.h_bw.set_data(disp_bw)
        plt.pause(self.t)
