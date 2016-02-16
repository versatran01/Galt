from __future__ import print_function, division, absolute_import

import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max
from skimage.morphology import watershed

from scpye.bounding_box import bbox_area, extract_bbox
from scpye.region_props import (region_props_bw, clean_bw, fill_bw,
                                local_max_points, gray_from_bw)


class BlobAnalyzer(object):
    fruit_dtype = [('bbox', np.int, 4), ('num', np.int, 1)]

    def __init__(self, min_area=5, split=False):
        """
        :param min_area: minimum area to be consider a blob
        :param split: whether to split big blob to smaller ones or not
        :return:
        """
        self.min_area = min_area
        self.split = split
        self.area_thresh = 0

    def analyze(self, bw, v):
        """
        :param bw:
        :param v:
        :return: (fruits, bw)
        """
        bw_clean, blobs = self.clean(bw)
        v[bw_clean == 0] = 0

        areas = blobs['prop'][:, 0]
        self.area_thresh = np.mean(areas)
        fruits = [self.split_blob(blob, v) for blob in blobs]
        fruits = np.vstack(fruits)
        return fruits, bw_clean

    def clean(self, bw):
        """
        Clean up raw binary detection image and do contour analysis
        :param bw:
        :return:
        """
        bw = gray_from_bw(bw)
        # Clean binary image
        bw = clean_bw(bw)
        # Get regional properties and also remove very small blobs
        blobs, cntrs = region_props_bw(bw, self.min_area)
        # Redraw bw with cntrs
        bw = fill_bw(bw, cntrs)
        return bw, blobs

    def split_blob(self, blob, v, min_aspect=1.4, max_extent=0.5):
        """
        :param blob:
        :param v:
        :param min_aspect:
        :param max_extent:
        :return:
        """
        bbox = blob['bbox']
        v_bbox = extract_bbox(v, bbox)

        min_dist = min(np.sqrt(bbox_area(bbox)) / 5, 10)
        area, aspect, extent = blob['prop']
        if area > self.area_thresh and \
                (aspect > min_aspect or extent < max_extent):
            points = find_local_maximas(v_bbox, min_distance=min_dist)

            if points is None:
                return np.hstack((bbox, 1))

            num = len(points)
            if self.split:
                return self.split_local_max_points(points, bbox, area)
            else:
                return np.hstack((bbox, num))
        else:
            return np.hstack((bbox, 1))

    @staticmethod
    def split_local_max_points(points, bbox, area):
        """
        Split local max points into multiple fruits [bbox, num]
        :param points:
        :param bbox:
        :param area:
        :return:
        """
        xb, yb, _, _ = bbox
        num = len(points)
        a = np.sqrt(area / num)
        fruits = []
        for pt in points:
            x, y = pt
            fruit = np.array([x + xb - a / 2, y + yb - a / 2, a, a, 1])
            fruits.append(fruit)
        return np.vstack(fruits)


def find_local_maximas(image, min_distance=10):
    """
    Find points of local maximas from gray scale image
    :param image:
    :param min_distance:
    :return:
    """
    image_max = ndi.maximum_filter(image, size=3, mode='constant')
    local_max = peak_local_max(image_max, min_distance=min_distance,
                               indices=False, exclude_border=False)
    local_max = gray_from_bw(local_max)
    points = local_max_points(local_max)
    return points


def label_blob_watershed(bbox, bw, v, k=5.5, return_num=False):
    """
    :param bbox: bounding box
    :param bw: binary image
    :param v: gray scale image
    :param k: magic number
    :param return_num: return number of labels
    :return:
    """
    min_dist = np.sqrt(bbox_area(bbox)) / k

    v_bbox = extract_bbox(v, bbox, copy=True)
    bw_bbox = extract_bbox(bw, bbox, copy=True)
    v_bbox[bw_bbox == 0] = 0
    dist = ndi.distance_transform_edt(bw_bbox) * 2
    dist += v_bbox

    local_max = peak_local_max(dist, indices=False, min_distance=min_dist,
                               labels=bw_bbox)
    if np.count_nonzero(local_max) > 1:
        markers = ndi.label(local_max, structure=np.ones((3, 3)))[0]
        label = watershed(-dist, markers, mask=bw_bbox)
    else:
        label = bw_bbox
    if return_num:
        return label, np.max(label)
    else:
        return label
