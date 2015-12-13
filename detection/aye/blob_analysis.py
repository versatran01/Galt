from __future__ import print_function, division, absolute_import
import cv2
import numpy as np
from aye.bounding_box import extract_bbox


def thresh_blobs_area(blobs, area=50):
    return blobs[blobs['area'] > area]


def is_blob_multiple(blob):
    # For now just return false
    # Some learning algorithm goes here
    return False


def num_peaks_in_blob(blob, image):
    if is_blob_multiple(blob):
        bbox = blob['bbox']
        bbox_region = extract_bbox(image, bbox)
        num_peaks = num_local_maximas(bbox_region)
    else:
        num_peaks = 1
    return num_peaks


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


def clean_bw(bw, n=3, close=True):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))
    bw_open = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)

    if close:
        return cv2.morphologyEx(bw_open, cv2.MORPH_CLOSE, kernel=kernel)

    return bw_open


def fill_holes(cs, shape):
    bw_filled = np.zeros(shape, np.uint8)
    cv2.drawContours(bw_filled, cs, -1, color=255, thickness=-1)
    return bw_filled


def region_props(bw, do_clean=True, min_area=4):
    if do_clean:
        bw_clean = clean_bw(bw, close=False)
    else:
        bw_clean = np.array(bw, copy=True)

    # Detect contour
    cs, _ = cv2.findContours(bw_clean, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)

    # Redraw the contour on a new image to fill all the holes
    bw_filled = fill_holes(cs, bw.shape)

    # Assemble a list of Blobs
    blob_dtype = [('area', 'int32'),
                  ('bbox', '(4,)int32'),
                  ('bbox_area', 'int32'),
                  ('extent', 'float32'),
                  ('equiv_diameter', 'float32')]

    blobs = []
    for cnt in cs:
        m = cv2.moments(cnt)
        area = m['m00']
        # We only accept blob that is decently big
        # because somethings we will get a blob with an area of 0
        if area > min_area:
            bbox = cv2.boundingRect(cnt)
            x, y, w, h = bbox
            bbox_area = w * h
            extent = area / bbox_area
            equiv_diameter = np.sqrt(4 * area / np.pi)
            blob = np.array((area, bbox, bbox_area, extent, equiv_diameter),
                            dtype=blob_dtype)
            blobs.append(blob)
    blobs = np.array(blobs)

    # TODO: should we sort all blobs here?
    return blobs, bw_filled
