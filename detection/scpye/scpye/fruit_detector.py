from __future__ import (print_function, division, absolute_import)
import numpy as np
from sklearn.externals import joblib
from sklearn.svm import SVC
from scpye.image_pipeline import (ImagePipeline, FeatureUnion)
from scpye.image_transformer import *


def make_image_pipeline(ccw=-1, bbox=None, v_min=25, cspace=None, use_loc=True):
    """
    Factory function for making an image pipeline
    :param ccw: rotate image by ccw
    :param bbox: crop image by bbox
    :param v_min: remove dark pixels < v_min
    :param cspace: features - colorspace
    :param use_loc: features - pixel location
    :return: image pipeline
    :rtype: ImagePipeline
    """
    features = make_feature_union(cspace, use_loc)

    img_ppl = ImagePipeline([
        ('rotate_image', ImageRotator(ccw)),
        ('crop_image', ImageCropper(bbox)),
        ('resize_image', ImageResizer()),
        ('remove_dark', DarkRemover(v_min)),
        ('features', features),
        ('scale', StandardScaler()),
    ])
    return img_ppl


def make_feature_union(cspace=None, use_loc=True):
    """
    Factory function for making a feature unin
    :param cspace: features - colorspace
    :param use_loc: features - pixel location
    :return: feature union
    :rtype: FeatureUnion
    """
    if cspace is None:
        cspace = ['bgr', 'hsv']
    transformer_list = [(cs, CspaceTransformer(cs)) for cs in cspace]
    if use_loc:
        transformer_list.append(('mask_location', MaskLocator()))

    return FeatureUnion(transformer_list, n_jobs=len(transformer_list))


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
