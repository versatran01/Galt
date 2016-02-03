from __future__ import (print_function, absolute_import, division)
from sklearn.svm import SVC
from sklearn.externals import joblib
from sklearn.grid_search import GridSearchCV
from scpye.image_transformer import (ImageRotator, ImageCropper, ImageResizer,
                                     DarkRemover, CspaceTransformer,
                                     MaskLocator, StandardScaler)
from scpye.image_pipeline import ImagePipeline, FeatureUnion


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


def train():
    pass
