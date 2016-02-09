from __future__ import print_function, division, absolute_import

import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max
from skimage.morphology import watershed

from scpye.bounding_box import bbox_area, extract_bbox
from scpye.region_props import region_props_bw, clean_bw, fill_bw


class BlobAnalyzer(object):
    def __init__(self, min_area=9, num_split=10):
        self.min_area = min_area
        self.num_split = num_split

    def analyze(self, bw, v):
        """

        :param bw:
        :param v:
        :return:
        """
        # Clean binary image
        bw = clean_bw(bw)
        # Get regional properties and also remove very small blobs
        blobs, cntrs = region_props_bw(bw, self.min_area)
        # Redraw bw with cntrs
        bw = fill_bw(bw, cntrs)

        # Partition blobs by area
        n_blobs = len(blobs)
        blobs = np.partition(blobs, n_blobs - self.num_split, order='area')

        # For small blobs just add it to output list
        bboxes = [blobs[:-self.num_split]['bbox']]

        for blob in blobs[-self.num_split:]:
            bbox = blob['bbox']
            label, n = label_blob(bbox, bw, v, return_num=True)
            if n == 1:
                # Nothing to split, just add to list
                bboxes.append(bbox)
            else:
                splited_bboxes = split_label(label)
                bboxes.extend(bboxes)

        return bboxes


def split_label():
    pass


def label_blob(bbox, bw, v, k=5.5, return_num=False):
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
    dist = ndi.distance_transform_edt(bw_bbox) * k
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
