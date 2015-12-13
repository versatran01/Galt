from __future__ import print_function, division, absolute_import
import cv2
import numpy as np


def num_local_maximas(image, n=7):
    """
    http://answers.opencv.org/question/28035/find-local-maximum-in-1d-2d-mat/
    :param image:
    :param n:
    :return:
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    peak = cv2.dilate(image, kernel, iterations=1)
    peak -= image

    flat = cv2.erode(image, kernel, iterations=1)
    flat = image - flat

    peak[peak > 0] = 255
    flat[flat > 0] = 255

    flat = cv2.bitwise_not(flat)
    peak[flat > 0] = 255
    peak = cv2.bitwise_not(peak)

    cs, _ = cv2.findContours(peak, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)
    return len(cs)


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
        bw_copy = np.array(bw, copy=True)

    # Detect contour
    cs, _ = cv2.findContours(bw_copy, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)

    # Redraw the contour on a new image to fill all the holes
    bw_filled = fill_holes(bw_copy, cs)

    # Assemble a list of Blobs
    dtype = [('area', 'int32'),
             ('bbox', '(4,)int32'),
             ('bbox_area', 'int32'),
             ('extent', 'float32'),
             ('equiv_diameter', 'float32')]

    stats = []
    for cnt in cs:
        m = cv2.moments(cnt)
        area = m['m00']
        if area > 0:
            bbox = cv2.boundingRect(cnt)
            bbox_area = bbox[-1] * bbox[-2]
            extent = area / bbox_area
            equiv_diameter = np.sqrt(4 * area / np.pi)
            stat = np.array((area, bbox, bbox_area, extent, equiv_diameter),
                            dtype=dtype)
            stats.append(stat)
    stats = np.array(stats)
    return stats, bw_filled
