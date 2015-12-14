from __future__ import print_function, division, absolute_import

import numpy as np
from aye.bounding_box import bbox_center
from aye.kalman_filter import KalmanFilter


class FruitTrack(object):
    def __init__(self, blob, num_fruits=1):
        self.blob = blob
        self.num_fruits = num_fruits
        self.kf = KalmanFilter(bbox_center(blob['bbox']))

    def get_bbox(self):
        return self.blob['bbox']

    def get_bbox_filtered(self):
        cx, cy = self.kf.x
        _, _, w, h = self.blob['bbox']
        x = cx - w / 2
        y = cy - h / 2
        return np.array([x, y, w, h], int)

    def get_x(self):
        return self.kf.x

    def get_age(self):
        return self.kf.length

    x = property(get_x)
    age = property(get_age)
    bbox = property(get_bbox)
    bbox_filtered = property(get_bbox_filtered)

    def predict(self, flow):
        self.kf.predict(flow)

    def correct(self, blob):
        # Update blob
        self.blob = blob
        z = bbox_center(self.bbox)
        self.kf.correct(z)
