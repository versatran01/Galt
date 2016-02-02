from __future__ import (print_function, division, absolute_import)
import numpy as np
from sklearn.externals import joblib
from scpye.image_pipeline import (ImagePipeline, FeatureUnion)
from scpye.image_transformer import *


class FruitDetector(object):
    def __init__(self, ppl):
        """
        :param ppl: image pipeline
        :type ppl: ImagePipeline
        """
        self.ppl = ppl

    def detect(self, img_raw):
        # TODO: what else to return from ppl?
        y = self.ppl.predict(img_raw)
        bw = np.array(self.ppl.named_steps['remove_dark'].mask, copy=True)
        bw[bw > 0] = y
        return bw

    @classmethod
    def from_pickle(cls, fname):
        """
        Constructor from a pickle
        :param fname:
        :type fname: str
        :return:
        :rtype: FruitDetector
        """
        return cls(joblib.load(fname))
