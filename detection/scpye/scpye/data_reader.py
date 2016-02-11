import os
import cv2
import numpy as np
from sklearn.externals import joblib

import rosbag
from cv_bridge import CvBridge, CvBridgeError


class DataReader(object):
    def __init__(self, base_dir='/home/chao/Workspace/bag', fruit='apple',
                 color='red', mode='fast_flash', bag='rect_fixed',
                 filename='frame{0:04d}_{1}.png',
                 topic='/color/image_rect_color'):
        self.base_dir = base_dir
        self.fruit = fruit
        self.color = color
        self.mode = mode
        self.filename = filename
        self.bagname = 'frame{0}_' + bag + '.bag'
        self.topic = topic

        # Directory
        self.data_dir = os.path.join(self.base_dir, fruit, color, mode)
        self.train_dir = os.path.join(self.data_dir, 'train')
        self.model_dir = os.path.join(self.data_dir, 'model')
        self.image_dir = os.path.join(self.data_dir, 'image')
        self.bag_dir = os.path.join(self.data_dir, bag)

    def _read_image(self, index, suffix, color=True):
        """
        Read image
        :param index: index of image
        :param suffix: suffix of image
        :param color: color or gray
        :return: image
        :rtype: numpy.ndarray
        """
        filename = os.path.join(self.train_dir,
                                self.filename.format(index, suffix))
        if color:
            flag = cv2.IMREAD_COLOR
        else:
            flag = cv2.IMREAD_GRAYSCALE
        image = cv2.imread(filename, flag)
        if image is None:
            raise ValueError('{0} not found'.format(filename))
        return image

    def load_image(self, index):
        """
        Load image by index
        :param index:
        :return:
        """
        return self._read_image(index, 'raw', color=True)

    def load_label(self, index):
        """
        Load label by index
        :param index:
        :return:
        """
        neg = self._read_image(index, 'neg', color=False)
        pos = self._read_image(index, 'pos', color=False)
        label = np.dstack((neg, pos))
        return label

    def load_image_label(self, index):
        """
        :param index: index of image
        :return: image and label
        """
        image = self.load_image(index)
        label = self.load_label(index)
        return image, label

    def save_model(self, model, name):
        """
        Save model to model directory
        :param model:
        :param name:
        :return:
        """
        model_pickle = os.path.join(self.model_dir, name + '.pkl')
        joblib.dump(model, model_pickle)
        print('{0} saved to {1}'.format(name, model_pickle))

    def load_model(self, name):
        """
        Load model from model directory
        :param name:
        :return:
        """
        model_pickle = os.path.join(self.model_dir, name + '.pkl')
        model = joblib.load(model_pickle)
        print('{0} load from {1}'.format(name, model_pickle))
        return model

    def load_image_label_list(self, image_indices):
        """
        Load image and label in separate lists
        :param image_indices:
        """

        # image_indices has to be a list
        if np.isscalar(image_indices):
            image_indices = [image_indices]

        Is = []
        Ls = []
        for ind in image_indices:
            I, L = self.load_image_label(ind)
            Is.append(I)
            Ls.append(L)

        return Is, Ls

    def load_bag(self, index):
        bagname = os.path.join(self.bag_dir, self.bagname.format(index))
        bridge = CvBridge()
        with rosbag.Bag(bagname) as bag:
            for topic, msg, t in bag.read_messages(self.topic):
                try:
                    image = bridge.imgmsg_to_cv2(msg)
                except CvBridgeError as e:
                    print(e)
                    continue
                yield image
