import os
import cv2
import numpy as np


class DataReader(object):
    def __init__(self, base_dir='/home/chao/Workspace/bag', fruit='apple',
                 color='red', mode='fast_flash', fname='frame{0:04d}_{1}.png'):
        self.base_dir = base_dir
        self.fruit = fruit
        self.color = color
        self.mode = mode
        self.fname = fname

        # Directory
        self.data_dir = os.path.join(self.base_dir, fruit, color, mode)
        self.train_dir = os.path.join(self.data_dir, 'train')

    def read_image(self, ind, suffix, color=True):
        """
        Read image
        :param ind: index of image
        :param suffix: suffix of image
        :param color: color or gray
        :return: image
        :rtype: numpy.ndarray
        """
        fname = os.path.join(self.train_dir, self.fname.format(ind, suffix))
        if color:
            flag = cv2.IMREAD_COLOR
        else:
            flag = cv2.IMREAD_GRAYSCALE
        image = cv2.imread(fname, flag)
        if image is None:
            raise ValueError('{0} not found'.format(fname))
        return image

    def load_image(self, ind):
        """
        Load image by index
        :param ind:
        :return:
        """
        return self.read_image(ind, 'raw', color=True)

    def load_label(self, ind):
        """
        Load label by index
        :param ind:
        :return:
        """
        neg = self.read_image(ind, 'neg', color=False)
        pos = self.read_image(ind, 'pos', color=False)
        label = np.dstack((neg, pos))
        return label

    def load_image_label(self, ind):
        """
        :param ind: index of image
        :return: image and label
        """
        image = self.load_image(ind)
        label = self.load_label(ind)
        return image, label

