from __future__ import print_function, division, absolute_import
import cv2
import os
import numpy as np


def resize_image(image, k=0.25):
    """
    Resize image using cv2.resize with a single scale k
    :param image: array-like
    :param k: float
    :return: array-like
    """
    return cv2.resize(image, None, fx=k, fy=k, interpolation=cv2.INTER_LINEAR)


def convert_image_colorspace(image, to):
    """
    Covert image colorspace from BGR to HSV or LAB
    :param image: array-like
    :param to: str
    :return: array-like
    """
    if to.lower() == 'hsv':
        flag = cv2.COLOR_BGR2HSV
    elif to.lower() == 'lab':
        flag = cv2.COLOR_BGR2LAB
    else:
        raise ValueError('colorspace {0} not supported'.format(to))

    return cv2.cvtColor(image, flag)


def image_to_feature_vector():
    pass


class DataReader(object):
    def __init__(self, rel_dir="../data", ext="png"):
        cwd = os.getcwd()
        self.dir = os.path.join(cwd, rel_dir)
        self.ext = ext
        self.fmt_image = "frame{0:04d}.{1}"
        self.fmt_label = "frame{0:04d}_{1}.{2}"

    def read_color_image(self, fid):
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
        image = self.read_color_image(fid)
        pos = cv2.imread(self.get_label_filename(fid, 'positive'), flag)
        neg = cv2.imread(self.get_label_filename(fid, 'negative'), flag)

        return image, [pos, neg]

    def read_cspace_with_label(self, fid, k=0.4):
        im_bgr, labels = self.read_image_with_label(fid)
        # Convert colorspace
        im_hsv = convert_image_colorspace(im_bgr, 'hsv')
        im_lab = convert_image_colorspace(im_bgr, 'lab')
        # Resize images
        images = [im_bgr, im_hsv, im_lab]
        images = [resize_image(im, k) for im in images]
        # Resize labels
        labels = [resize_image(im, k) for im in labels]
        return images, labels


def make_train_samples_features(images, labels, v_thresh=None):
    im_bgr, im_hsv, im_lab = images
    im_pos, im_neg = labels

    if v_thresh:
        v = im_hsv[:, :, -1]
        mask_v = (v > v_thresh) & (v < 255 - v_thresh)
    else:
        mask_v = np.ones_like(im_pos) > 0

    mask_pos = im_pos > 0
    mask_neg = im_neg > 0

    mask_pos_v = mask_pos & mask_v
    mask_neg_v = mask_neg & mask_v

    X_pos_bgr = im_bgr[mask_pos_v]
    X_pos_hsv = im_hsv[mask_pos_v]
    X_pos_lab = im_lab[mask_pos_v]

    X_neg_bgr = im_bgr[mask_neg_v]
    X_neg_hsv = im_hsv[mask_neg_v]
    X_neg_lab = im_lab[mask_neg_v]

    X_pos = np.hstack((X_pos_bgr, X_pos_hsv, X_pos_lab))
    X_neg = np.hstack((X_neg_bgr, X_neg_hsv, X_neg_lab))

    # Convert to float
    X_pos = np.array(X_pos, float)
    X_neg = np.array(X_neg, float)

    # Make sure we don't have too many negative samples than positive samples
    if len(X_neg) / len(X_pos) > 2:
        X_neg = X_neg[::2]

    # Generate corresponding labels
    y_pos = np.ones((np.size(X_pos, axis=0)))
    y_neg = np.zeros((np.size(X_neg, axis=0)))

    X_both = np.vstack((X_pos, X_neg))
    y_both = np.hstack((y_pos, y_neg))

    return X_both, y_both


def make_test_samples(images):
    im_bgr, im_hsv, im_lab = images

    h, w, _ = im_bgr.shape
    X_bgr = np.reshape(im_bgr, (h * w, -1))
    X_hsv = np.reshape(im_hsv, (h * w, -1))
    X_lab = np.reshape(im_lab, (h * w, -1))

    X = np.hstack((X_bgr, X_hsv, X_lab))
    X = np.array(X, float)
    return X


def make_masked_test_samples(images, v_thresh):
    im_bgr, im_hsv, im_lab = images

    v = im_hsv[:, :, -1]
    mask = (v > v_thresh) & (v < 255 - v_thresh)

    X_bgr = im_bgr[mask]
    X_hsv = im_hsv[mask]
    X_lab = im_lab[mask]

    X = np.hstack((X_bgr, X_hsv, X_lab))
    X = np.array(X, float)

    return X, mask


def masked_y_to_image(y, mask):
    h, w = mask.shape
    mask_vec = np.reshape(mask, (-1,))
    idx = np.array(np.where(mask_vec)).ravel()
    idx = idx[y > 0]
    bw = np.zeros((h * w,))
    bw[idx] = 1
    bw = np.reshape(bw, (h, w))
    return bw
