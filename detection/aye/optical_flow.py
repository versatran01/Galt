from __future__ import print_function, division, absolute_import
import cv2
import numpy as np


def points_inside_image(points, image, bound=4):
    h, w = image.shape
    px = points[:, :, 0]
    py = points[:, :, 1]
    return (px >= bound) & (px < w - bound) & (py >= bound) & (py < h - bound)


def calc_average_flow(flows, sts=None):
    if sts is None:
        flows_good = flows
    else:
        flows_good = flows[sts > 0]
    return np.mean(flows_good, axis=0)


def calc_bboxes_flow(gray1, gray2, bboxes1, win_size=21, max_level=3,
                     guess=None):
    p1 = []
    for bbox in bboxes1:
        x, y, w, h = bbox
        p1.append(np.array([x + w / 2, y + h / 2]))
    # OpenCV need np.float32
    p1 = np.array(p1, np.float32)
    p1 = p1[:, np.newaxis, :]

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    if guess is not None:
        p2_0 = np.array(p1, copy=True)
        p2_0 += guess
        klt_params = dict(winSize=(win_size, win_size),
                          maxLevel=max_level,
                          flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                          criteria=criteria)
        p2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, p1, p2_0,
                                               **klt_params)
    else:
        klt_params = dict(winSize=(win_size, win_size),
                          maxLevel=max_level,
                          criteria=criteria)
        p2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, p1, None,
                                               **klt_params)

    # we also check whether p2 is inside some bounds of the image, 4 pixels
    is_inside = points_inside_image(p2, gray2)
    st = (st == 1) & is_inside

    return p1, p2, st
