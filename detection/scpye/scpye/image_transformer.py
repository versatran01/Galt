from __future__ import (absolute_import, division, print_function)
import cv2
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.base import TransformerMixin, BaseEstimator
from functools import partial

from scpye.bounding_box import extract_bbox

__all__ = ['ImageRotator', 'ImageCropper', 'ImageResizer']


class ImageTransformer(BaseEstimator, TransformerMixin):
    """
    Base class for image transformation
    """

    def fit(self, X, y=None):
        return self

    @staticmethod
    def _transform(X, func):
        """
        Apply transform to X return None if X is None
        :param X: data
        :param func: function to apply to data
        :return: transformed data
        """
        if X is None:
            return None

        return func(X)


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
        :param y: label
        :return: rotated image and label
        """
        func = partial(np.rot90, k=self.ccw)
        Xt = self._transform(X, func)
        yt = self._transform(y, func)
        return Xt, yt


class ImageCropper(ImageTransformer):
    def __init__(self, bbox=None):
        self.bbox = bbox

    def transform(self, X, y=None):
        """
        :param X: image
        :param y: label
        :return: region of image and label
        """
        func = partial(extract_bbox, bbox=self.bbox)
        Xt = self._transform(X, func)
        yt = self._transform(y, func)
        return Xt, yt


class ImageResizer(ImageTransformer):
    def transform(self, X, y=None):
        """
        :param X: image
        :return: resized image
        """
        k = 0.5
        func_x = cv2.pyrDown
        func_y = partial(cv2.resize, dsize=None, fx=k, fy=k,
                         interpolation=cv2.INTER_NEAREST)
        Xt = self._transform(X, func_x)
        yt = self._transform(y, func_y)
        return Xt, yt


class DarkPixelRemover(ImageTransformer):
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
        :return: a tuple of bgr image and mask
        """
        img_hsv = cv2.cvtColor(X, cv2.COLOR_BGR2HSV)
        self.mask = img_hsv[:, :, -1] > self.v_min


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
        :return: masked image with transformed colorspace
        """
        bgr, mask = X
        self.img = self.cspace_transform(bgr)
        return np.array(self.img[mask], dtype=np.float64)
