from __future__ import print_function, division, absolute_import
import cv2
import numpy as np


def clean_bw(bw, n=3):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    bw_clean = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)
    return bw_clean


def fill_holes(bw, cs):
    bw_filled = np.zeros_like(bw)
    cv2.drawContours(bw_filled, cs, -1, color=255, thickness=-1)
    return bw_filled


def region_props(bw, do_clean=True):
    if do_clean:
        bw_copy = clean_bw(bw)
    else:
        bw_copy = np.array(bw, )

    # Detect contour
    cs, _ = cv2.findContours(bw_copy, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)

    # Redraw the contour on a new image to fill all the holes
    bw_filled = fill_holes(bw_copy, cs)

    # Assemble a list of Blobs
    bboxes = []
    for cnt in cs:
        x, y, w, h = cv2.boundingRect(cnt)
        bboxes.append(np.array([x, y, w, h]))

    bboxes = np.array(bboxes, dtype=float)
    return bboxes, bw_filled
