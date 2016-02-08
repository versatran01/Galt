from __future__ import print_function, division, absolute_import
import cv2
import numpy as np

"""
http://scikit-image.org/docs/dev/api/skimage.measure.html#skimage.measure.regionprops
"""

blob_dtype = [('area', 'int32'), ('bbox', '(4,)int32'),
              ('extent', 'float32'), ('equiv_diameter', 'float32'),
              ('solid', 'float32'), ('axes_ratio', 'float32'),
              ('eccen', 'float32')]


def region_props(bw):
    """
    Same as matlab regionprops but implemented in opencv
    :param bw: binary image
    :return: a list of blobs
    """
    # Need to do a copy because opencv find contour will modify image
    cntrs_raw = find_contours(bw)

    blobs = []
    cntrs = []
    for cntr in cntrs_raw:
        m = cv2.moments(cntr)
        area = m['m00']
        if area >= 4 and len(cntr) >= 5:
            # Bbox
            bbox = np.array(cv2.boundingRect(cntr))
            bbox_area = bbox[-1] * bbox[-2]
            extent = area / bbox_area
            equiv_diameter = np.sqrt(4 * area / np.pi)
            # Convex
            cvx_hull = cv2.convexHull(cntr)
            cvx_area = cv2.contourArea(cvx_hull)
            # Solidity
            solid = area / cvx_area
            # Ellipse
            center, axes, angle = cv2.fitEllipse(cntr)
            maj_ind = np.argmax(axes)
            maj_axes = axes[maj_ind]
            min_axes = axes[1 - maj_ind]
            axes_ratio = min_axes / maj_axes
            # Eccentricity
            eccen = np.sqrt(1 - axes_ratio ** 2)

            # blob
            blob = np.array((area, bbox, extent, equiv_diameter,
                             solid, axes_ratio, eccen),
                            dtype=blob_dtype)
            blobs.append(blob)
            cntrs.append(cntr)

    blobs = np.array(blobs)
    return blobs, cntrs


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


def moment_centroid(mmt):
    return np.array([mmt['m10'] / mmt['m00'], mmt['m01'] / mmt['m00']])


def morph_opening(bw, ksize=3, iters=1):
    """
    http://docs.opencv.org/2.4/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html
    http://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html#gsc.tab=0
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
    http://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html#gsc.tab=0
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
