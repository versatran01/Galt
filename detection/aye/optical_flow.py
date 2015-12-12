from __future__ import print_function, division, absolute_import
import cv2
import numpy as np
import matplotlib.pyplot as plt


def calc_flow(gray1, gray2, bboxes1, win_size=21, max_level=3, guess=None):
    p1 = []
    for bbox in bboxes1:
        x, y, w, h = bbox
        p1.append(np.array([x + w / 2, y + h / 2]))
    # OpenCV need np.float32
    p1 = np.array(p1, np.float32)
    p1 = p1[:, np.newaxis, :]

    if guess is not None:
        p2_0 = np.array(p1, copy=True)
        p2_0 += guess
        klt_params = dict(winSize=(win_size, win_size),
                          maxLevel=max_level,
                          flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                          criteria=(
                              cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                              10, 0.03))
        p2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, p1, p2_0,
                                               **klt_params)
    else:
        klt_params = dict(winSize=(win_size, win_size),
                          maxLevel=max_level,
                          criteria=(
                              cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                              10, 0.03))
        p2, st, err = cv2.calcOpticalFlowPyrLK(gray1, gray2, p1, None,
                                               **klt_params)

    st = (st == 1)
    return np.squeeze(p1), np.squeeze(p2), np.squeeze(st)
