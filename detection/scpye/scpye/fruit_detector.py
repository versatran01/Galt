from __future__ import (print_function, division, absolute_import)
import numpy as np
from sklearn.externals import joblib
from sklearn.svm import SVC
from scpye.image_pipeline import (ImagePipeline, FeatureUnion)
from scpye.image_transformer import *


class FruitDetector(object):
    def __init__(self, ppl, clf):
        """
        :param ppl: image pipeline
        :type ppl: ImagePipeline
        :param clf: classifier
        """
        self.ppl = ppl
        self.clf = clf

    def detect(self, img_raw):
        # TODO: what else to return from ppl?
        Xt = self.ppl.transform(img_raw)
        y = self.clf.predict(Xt)
        bw = np.array(self.ppl.named_steps['remove_dark'].mask, copy=True)
        bw[bw > 0] = y
        return bw

    @classmethod
    def from_pickle(cls, ppl_file, clf_file):
        """
        Constructor from a pickle
        :param clf_file:
        :param ppl_file:
        :return:
        :rtype: FruitDetector
        """
        ppl = joblib.load(ppl_file)
        clf = joblib.load(clf_file)
        return cls(ppl, clf)
