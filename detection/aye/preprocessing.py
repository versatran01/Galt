from __future__ import print_function, division, absolute_import
import cv2


def resize_image(image, k=0.25):
    """
    Resize image using cv2.resize with a single scale k
    :param image:
    :param k:
    :return:
    """
    return cv2.resize(image, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
