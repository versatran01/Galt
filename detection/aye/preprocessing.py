from __future__ import print_function, division, absolute_import
import cv2


def resize_image(image, k=0.25):
    """
    Resize image using cv2.resize with a single scale k
    :param image: array-like
    :param k: float
    :return: array-like
    """
    return cv2.resize(image, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)


def convert_image_colorspace(image, to):
    """
    Covert image colorspace from BGR to HSV or LAB
    :param image: array-like
    :param to: str
    :return: array-like
    """
    if to.lower() == 'hsv':
        flag = cv2.COLOR_BGR2HSV
    elif to.lower() == 'lab':
        flag = cv2.COLOR_BGR2LAB
    else:
        raise ValueError('colorspace {0} not supported'.format(to))

    return cv2.cvtColor(image, flag)


def image_to_feature_vector():
    pass
