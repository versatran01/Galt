from __future__ import print_function, division, absolute_import
import cv2
import os
import numpy as np


def resize_image(image, k=0.25, interpolation=cv2.INTER_NEAREST):
    return cv2.resize(image, None, fx=k, fy=k, interpolation=interpolation)


def prepare_data(reader, file_ids):
    X = None
    y = None
    for fid in file_ids:
        image, labels = reader.read_image_with_label(fid)
        s = Samples(image, labels)
        X_both, y_both = s.Xy_both()

        if X is None or y is None:
            X = X_both
            y = y_both
        else:
            X = np.vstack((X, X_both))
            y = np.hstack((y, y_both))

    return X, y


def convert_image_colorspace(image, to):
    if to.lower() == 'hsv':
        flag = cv2.COLOR_BGR2HSV
    elif to.lower() == 'lab':
        flag = cv2.COLOR_BGR2LAB
    else:
        raise ValueError('colorspace {0} not supported'.format(to))

    return cv2.cvtColor(image, flag)


class DataReader(object):
    def __init__(self, rel_dir="../data", ext="png"):
        cwd = os.getcwd()
        self.dir = os.path.join(cwd, rel_dir)
        self.ext = ext
        self.fmt_image = "frame{0:04d}.{1}"
        self.fmt_label = "frame{0:04d}_{1}.{2}"

    def read_image(self, fid):
        image_name = self.fmt_image.format(fid, self.ext)
        filename = os.path.join(self.dir, image_name)
        image = cv2.imread(filename, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("{0} not found".format(filename))
        return image

    def get_label_filename(self, fid, label):
        if label not in ('positive', 'negative'):
            raise ValueError("Label '{0}' is not valid'".format(label))
        label_name = self.fmt_label.format(fid, label, self.ext)
        return os.path.join(self.dir, label_name)

    def read_image_with_label(self, fid):
        flag = cv2.IMREAD_GRAYSCALE
        image = self.read_image(fid)
        pos = cv2.imread(self.get_label_filename(fid, 'positive'), flag)
        neg = cv2.imread(self.get_label_filename(fid, 'negative'), flag)

        return image, [pos, neg]


def rotate_image(image):
    image = cv2.transpose(image)
    image = cv2.flip(image, 1)
    return image


class Samples(object):
    k = 0.25
    v_thresh = 25

    def __init__(self, im_bgr, labels=None):
        # Get images of all color spaces
        # For images we resize using linear
        self.im_raw = resize_image(im_bgr, self.k, cv2.INTER_LINEAR)
        # Blur the image a bit
        self.im_bgr = cv2.GaussianBlur(self.im_raw, (5, 5), 1)
        self.im_hsv = convert_image_colorspace(self.im_bgr, "hsv")
        self.im_lab = convert_image_colorspace(self.im_bgr, "lab")

        # Handle labels, if labels is provided, we are making training samples
        # otherwise we are making testing samples and generally don't care
        # about labels
        self.pos = None
        self.neg = None
        if labels:
            assert len(labels) == 2
            im_pos, im_neg = labels
            # For labels we resize using nearest because they are masks
            im_pos = resize_image(im_pos, self.k, cv2.INTER_NEAREST)
            im_neg = resize_image(im_neg, self.k, cv2.INTER_NEAREST)
            self.pos = im_pos > 0
            self.neg = im_neg > 0

        # Mask out invalid data based on v value in hsv
        v = self.im_hsv[:, :, -1]
        self.mask = (v >= self.v_thresh) & (v <= 255 - self.v_thresh)

    def extract(self, label):
        m = label & self.mask
        X_bgr = self.im_bgr[m]
        X_hsv = self.im_hsv[m]
        X_lab = self.im_lab[m]
        X = np.hstack((X_bgr, X_hsv, X_lab))
        n_samples, n_features = X.shape
        assert n_features == 9
        # Convert to float
        X = np.array(X, float)
        return X

    def Xy_pos(self):
        assert self.pos is not None

        X_pos = self.extract(self.pos)
        y_pos = np.ones((len(X_pos),))
        return X_pos, y_pos

    def Xy_neg(self):
        assert self.neg is not None

        X_neg = self.extract(self.neg)
        y_neg = np.zeros((len(X_neg),))
        return X_neg, y_neg

    def Xy_both(self, balance=True):
        X_pos, y_pos = self.Xy_pos()
        X_neg, y_neg = self.Xy_neg()
        if balance:
            if len(X_neg) / len(X_pos) > 2:
                X_neg = X_neg[::2]
                y_neg = y_neg[::2]
        X = np.vstack((X_pos, X_neg))
        y = np.hstack((y_pos, y_neg))

        return X, y

    def X(self):
        h, w, _ = self.im_bgr.shape
        X_bgr = np.reshape(self.im_bgr, (h * w, -1))
        X_hsv = np.reshape(self.im_hsv, (h * w, -1))
        X_lab = np.reshape(self.im_lab, (h * w, -1))

        m = np.reshape(self.mask, (-1,))
        X = np.hstack((X_bgr, X_hsv, X_lab))
        X = X[m]
        X = np.array(X, float)
        return X

    def y_to_bw(self, y, to_gray=False):
        h, w = self.mask.shape
        m = np.reshape(self.mask, (-1,))
        idx = np.array(np.where(m))
        idx = idx.ravel()
        idx = idx[y > 0]
        bw = np.zeros((h * w,))
        bw[idx] = 1
        bw = np.reshape(bw, (h, w))

        if to_gray:
            bw = np.array(bw, np.uint8)
            bw *= 255

        return bw