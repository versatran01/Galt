from __future__ import (print_function, division, absolute_import)

import cv2
import numpy as np


def points_inside_image(points, image, b=4):
    """
    Check if point is inside image with a certain margin
    :param points:
    :param image:
    :param b: distance from border
    :return:
    """
    h, w = image.shape
    px = points[:, :, 0]
    py = points[:, :, 1]
    return (px >= b) & (px < w - b) & (py >= b) & (py < h - b)


def _prepare_points(points):
    """
    Prepare points for opencv
    :param points:
    :return:
    """
    if points.dtype is not np.float32:
        points = np.array(points, dtype=np.float32)
    if np.ndim(points) == 2:
        points = np.expand_dims(points, 1)
    return points


def calc_optical_flow(gray1, gray2, points1, points2, win_size, max_level):
    """
    Thin wrapper around opencv's calcOpticalFlowPyrLK
    :param gray1:
    :param gray2:
    :param points1:
    :param points2:
    :param win_size:
    :param max_level:
    :return:
    """
    points1 = _prepare_points(points1)
    points2 = _prepare_points(points2)

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    klt_params = dict(winSize=(win_size, win_size),
                      maxLevel=max_level,
                      flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                      criteria=criteria)

    points2, status, _ = cv2.calcOpticalFlowPyrLK(gray1, gray2, points1,
                                                  points2, **klt_params)

    is_inside = points_inside_image(points2, gray2)

    status = (status == 1) & is_inside

    return points1, points2, np.squeeze(status)
