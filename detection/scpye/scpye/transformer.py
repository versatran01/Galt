from __future__ import (absolute_import, division, print_function)
import cv2
import numpy as np
from sklearn.base import TransformerMixin

from scpye.bounding_box import extract_bbox


class ImageTransformer(TransformerMixin):
    """
    Base class for image transformation
    """

    def fit(self, X, y=None):
        return self


class ImageRotator(ImageTransformer):
    def __init__(self, ccw=-1):
        """
        :param ccw: number of counter-clockwise rotations
        :type ccw: int
        """
        self.ccw = ccw

    def transform(self, X, y=None):
        """
        :param X: image
        :return: rotated image
        """
        assert np.ndim(X) >= 2
        return np.rot90(X, self.ccw)


class ImageCropper(ImageTransformer):
    def __init__(self, bbox=None):
        self.bbox = bbox

    def transform(self, X, y=None):
        """
        :param X: image
        :return: region of image
        """
        if self.bbox is None:
            return X
        else:
            return extract_bbox(X, self.bbox)


class ImageResizer(ImageTransformer):
    def __init__(self, k=0.5):
        """
        :param k: resize factor
        :type k: float
        """
        assert 0 < k <= 1
        self.k = k

    def transform(self, X, y=None):
        """
        :param X: image
        :return: resized image
        """
        if self.k == 0.5:
            return cv2.pyrDown(X)
        else:
            raise ValueError("not implemented")


class DarkRemover(ImageTransformer):
    def __init__(self, v_min=25):
        """
        :param v_min: minimum value in v channel
        :type v_min: int
        """
        assert 0 < v_min < 255
        self.mask = None
        self.v_min = v_min

    def transform(self, X, y=None):
        """
        :param X: bgr image
        :param y:
        :return: a tuple of bgr image and mask
        """
        img_hsv = cv2.cvtColor(X, cv2.COLOR_BGR2HSV)
        self.mask = img_hsv[:, :, -1] > self.v_min
        return X, self.mask


class CspaceTransformer(ImageTransformer):
    def __init__(self, des):
        self.des = des
        self.img = None

    def cspace_transform(self, bgr):
        """
        :param bgr: bgr image
        :return: bgr image in other colorspace
        """
        if self.des == 'hsv':
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        elif self.des == 'lab':
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        elif self.des == 'bgr':
            return bgr
        else:
            raise ValueError("{0} not supported".format(self.des))

    def transform(self, X, y=None):
        """
        :param X: tuple of bgr image and mask
        :return:
        """
        bgr, mask = X
        self.img = self.cspace_transform(bgr)
        return self.img[mask]
