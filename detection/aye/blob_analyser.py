from __future__ import print_function, division, absolute_import
import cv2


def clean_bw(bw, n=3):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    bw_clean = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)
    return bw_clean
