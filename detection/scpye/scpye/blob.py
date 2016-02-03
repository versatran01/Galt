from __future__ import print_function, division, absolute_import
import cv2
import numpy as np

"""
http://scikit-image.org/docs/dev/api/skimage.measure.html#skimage.measure.regionprops
"""


class BlobAnalyzer(object):
    def __init__(self):
        # Assemble a list of Blobs
        self.blob_dtype = [('area', 'int32'),
                           ('bbox', '(4,)int32'),
                           ('bbox_area', 'int32'),
                           ('extent', 'float32'),
                           ('equiv_diameter', 'float32')]

    def fill_holes(self, cs, shape):
        bw_filled = np.zeros(shape, np.uint8)
        cv2.drawContours(bw_filled, cs, -1, color=255, thickness=-1)
        return bw_filled

    # TODO: Investigate scikit-image regionprops
    def analyze(self, bw):
        # Detect contour
        cs, _ = cv2.findContours(bw, mode=cv2.RETR_EXTERNAL,
                                 method=cv2.CHAIN_APPROX_SIMPLE)

        # Redraw the contour on a new image to fill all the holes
        bw_filled = self.fill_holes(cs, bw.shape)

        blobs = []
        for cnt in cs:
            m = cv2.moments(cnt)
            area = m['m00']
            # We only accept blob that is decently big
            # because sometimes we will get a blob with an area of 0
            if area > 0:
                bbox = cv2.boundingRect(cnt)
                x, y, w, h = bbox
                bbox_area = w * h
                extent = area / bbox_area
                equiv_diameter = np.sqrt(4 * area / np.pi)
                blob = np.array((area, bbox, bbox_area, extent, equiv_diameter),
                                dtype=self.blob_dtype)
                blobs.append(blob)
        blobs = np.array(blobs)

        return blobs, bw_filled
