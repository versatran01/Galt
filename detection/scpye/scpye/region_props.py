from __future__ import (print_function, absolute_import, division)
import cv2
import numpy as np

blob_dtype = [('bbox', np.int, 4), ('prop', np.float, 3)]


def region_props_bw(bw, min_area=0):
    """
    Same as matlab regionprops but implemented in opencv
    Prefer using this than skimage's regionprops because this return a numpy
    recarray that is very compact
    :param bw: binary image
    :param min_area:
    :return: a structured array of blobs
    """
    contours = find_contours(bw)
    return region_props(contours, min_area=min_area)


def region_props(contours, min_area=0):
    """
    :param contours:
    :param min_area:
    :return:
    """
    blobs = []
    cntrs = []
    for cntr in contours:
        area = cv2.contourArea(cntr)
        # Need len(cntr) >= 5 to fit ellipse
        if area >= min_area:
            # Bbox
            bbox = np.array(cv2.boundingRect(cntr))
            _, _, w, h = bbox
            bbox_area = w * h
            aspect = w / h if w > h else h / w
            # Extent
            extent = area / bbox_area
            # Convex
            # cvx_hull = cv2.convexHull(cntr)
            # cvx_area = cv2.contourArea(cvx_hull)
            # Solidity
            # solid = area / cvx_area
            # Ellipse
            # center, axes, angle = cv2.fitEllipse(cntr)
            # maj_ind = np.argmax(axes)
            # maj_axes = axes[maj_ind]
            # min_axes = axes[1 - maj_ind]
            # axes_ratio = min_axes / maj_axes
            # Eccentricity
            # eccen = np.sqrt(1 - axes_ratio ** 2)

            # Assemble to recarray
            blob = np.array((bbox, (area, aspect, extent)), dtype=blob_dtype)
            blobs.append(blob)
            cntrs.append(cntr)

    blobs = np.array(blobs)
    return blobs, cntrs


def morph_opening(bw, ksize=3, iters=1):
    """
    :param bw: binary image
    :param ksize: kernel size
    :param iters: number of iterations
    :return: binary image after opening
    :rtype: numpy.ndarray
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_opened = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel,
                                 iterations=iters)
    return bw_opened


def morph_closing(bw, ksize=3, iters=1):
    """
    :param bw: binary image
    :param ksize: kernel size
    :param iters: number of iterations
    :return: binary image after closing
    :rtype: numpy.ndarray
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_closed = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel, iterations=iters)
    return bw_closed


def find_contours(bw, method=cv2.CHAIN_APPROX_NONE):
    """
    :param bw: binary image
    :param method:
    :return: a list of contours
    """
    contours, _ = cv2.findContours(bw.copy(), mode=cv2.RETR_EXTERNAL,
                                   method=method)
    return contours


def clean_bw(bw, ksize=3, iters=1):
    """
    Clean binary image by doing a opening followed by a closing
    :param bw: binary image
    :param ksize: kernel size
    :param iters: number of iterations
    :return: cleaned binary image
    """
    bw = morph_opening(bw, ksize=ksize, iters=iters)
    bw = morph_closing(bw, ksize=ksize, iters=iters)
    return bw


def fill_bw(bw, contours):
    """
    Redraw contours of binary image
    :param bw:
    :param contours:
    :return: filled image
    :rtype: numpy.ndarray
    """
    bw_filled = np.zeros_like(bw)
    cv2.drawContours(bw_filled, contours, -1, color=255, thickness=-1)

    return bw_filled


def gray_from_bw(bw, color=False):
    """
    Convert binary image (bool, int) to grayscale image (gray, bgr)
    :param bw: binary image
    :param color: whether to convert to bgr
    :return: grayscale image
    """
    gray = np.array(bw, dtype='uint8') * 255

    if color:
        bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return bgr
    else:
        return gray


def local_max_points(bw):
    """
    :param bw:
    :return:
    """
    cs = find_contours(bw)

    if len(cs) == 0:
        return None

    points = []
    for cnt in cs:
        m = cv2.moments(cnt)
        a = cv2.contourArea(cnt)
        if a > 0:
            points.append(moment_centroid(m))

    if len(points) == 0:
        return None
    else:
        points = np.array(points)
        return points


def moment_centroid(mmt):
    """
    Centroid of moment
    :param mmt:
    :return:
    """
    return np.array((mmt['m10'] / mmt['m00'], mmt['m01'] / mmt['m00']))
