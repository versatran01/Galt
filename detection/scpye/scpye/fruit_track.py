from __future__ import (print_function, division, absolute_import)

import numpy as np

from scpye.bounding_box import bbox_center
from scpye.kalman_filter import KalmanFilter


class FruitTrack(object):
    def __init__(self, fruit):
        self.bbox = fruit[:4]
        self.num = fruit[-1]
        self.kf = KalmanFilter(bbox_center(self.bbox))

    def get_bbox_filtered(self):
        cx, cy = self.kf.x
        _, _, w, h = self.bbox
        x = cx - w / 2
        y = cy - h / 2
        return np.array([x, y, w, h], dtype=np.int)

    @property
    def age(self):
        return self.kf.length

    def predict(self, flow):
        self.kf.predict(flow)

    def correct(self, fruit):
        # Update blob
        self.bbox = fruit[:4]
        self.num = fruit[-1]
        z = bbox_center(self.bbox)
        self.kf.correct(z)
