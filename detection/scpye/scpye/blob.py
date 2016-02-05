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
        """

        :param blobs:
        :param area:
        :return:
        :rtype:
        """
        return blobs[blobs['area'] > area]

    def is_blob_multiple(self, blob, min_area):
        """

        :param blob:
        :param min_area:
        :return:
        """
        return blob['area'] >= min_area

    def num_peaks_in_blob(self, blob, image, min_area=25):
        """

        :param blob:
        :param image:
        :param min_area:
        :return: number of peaks in blob
        :rtype: int
        """
        if self.is_blob_multiple(blob, min_area):
            bbox = blob['bbox']
            region = self.extract_bbox(image, bbox)
            # TODO: need to change n according to image size
            num_peaks = self.num_local_maximas(region)
        else:
            num_peaks = 1
        return num_peaks

    # TODO: replace this with ndimage stuff instead of using my own
    def num_local_maximas(self, image, n=7):
        """
        http://answers.opencv.org/question/28035/find-local-maximum-in-1d-2d-mat/
        :param image:
        :param n: int
        :return: number of local maximas
        :rtype: int
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

    def fill_holes(self, cs, shape):
        """

        :param cs:
        :param shape:
        :return: filled image
        :rtype: numpy.ndarray
        """
        bw_filled = np.zeros(shape, np.uint8)
        cv2.drawContours(bw_filled, cs, -1, color=255, thickness=-1)

        return bw_filled

    # TODO: Investigate scikit-image regionprops
    def analyze(self, bw):
        """

        :param bw: numpy.ndarray
        :return:
        """
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


def morph_opening(bw, ksize=3):
    """
    http://docs.opencv.org/2.4/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html
    http://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html#gsc.tab=0
    :param bw: binary image
    :type bw: numpy.ndarray
    :param ksize: kernel size
    :type ksize: int
    :return: binary image after opening
    :rtype: numpy.ndarray
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_open = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel=kernel)
    return bw_open


def morph_closing(bw, ksize=3):
    """
    :param bw:
    :type bw: numpy.ndarray
    :param ksize: kernel size
    :type ksize: int
    :return: binary image after closing
    :rtype: numpy.ndarray
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    bw_close = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel)
    return bw_close


def find_contours(bw):
    """
    http://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html#gsc.tab=0
    :param bw: binary image
    :type bw: numpy.ndarray
    :return: a list of contours
    :rtype: List
    """
    cs, _ = cv2.findContours(bw, mode=cv2.RETR_EXTERNAL,
                             method=cv2.CHAIN_APPROX_SIMPLE)
    return cs
