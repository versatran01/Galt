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

    def thresh_blobs_area(self, blobs, area):
        return blobs[blobs['area'] > area]

    def is_blob_multiple(self, blob, min_area):
        return blob['area'] >= min_area

    def num_peaks_in_blob(self, blob, image, min_area=25):
        if self.is_blob_multiple(blob, min_area):
            bbox = blob['bbox']
            region = extract_bbox(image, bbox)
            # TODO: need to change n according to image size
            num_peaks = num_local_maximas(region)
        else:
            num_peaks = 1
        return num_peaks

    def num_local_maximas(self, image, n=7):
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

    def clean_bw(self, bw, ksize=3):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        bw_open = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)

        return bw_open

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
