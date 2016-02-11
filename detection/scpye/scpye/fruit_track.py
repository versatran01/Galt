from __future__ import (print_function, division, absolute_import)

import numpy as np


class FruitTrack(object):
    fruit_dtype = [('bbox', np.int, 4), ('num', np.int)]

    def __init__(self, fruit, flow=np.zeros(2, np.int)):
        self.bbox = fruit[:4]
        self.num = fruit[-1]
        self.flow = flow
        self.age = 1

    def predict(self, flow):
        """
        Predict new location of the track
        Modifies the [x, y] of bbox
        :param flow:
        """
        self.bbox[:2] += np.array(flow, dtype=self.bbox.dtype)
        self.flow = flow

    def correct(self, fruit):
        """
        Correct location of the track
        :param fruit:
        """
        bbox_new = fruit[:4]
        self.num = fruit[-1]
        # Update flow first
        self.flow += (bbox_new[:2] - self.bbox[:2]) / 2
        # Then update bbox
        self.bbox = bbox_new
        # Increment age
        self.age += 1
