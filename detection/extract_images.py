#!/usr/bin/env python
from __future__ import print_function
import rosbag
import cv2
import sys
import os
import random
import argparse
from cv_bridge import CvBridge, CvBridgeError


def get_random_indices(n, total):
    return sorted(random.sample(range(0, total), n))


def extract_images(bagfile, output_dir, image_topic, num_images):
    with rosbag.Bag(bagfile, 'r') as bag:
        total_images = bag.get_message_count(image_topic)

        if total_images == 0:
            raise ValueError('total_images == 0 under topic ', image_topic)

        if num_images > total_images:
            raise ValueError('num_images {0} > total_images {1}'.format(
                num_images, total_images))

        # seq encodes which image to sample out of all images
        seq = get_random_indices(num_images, total_images)

        # Go through all messages in this topic
        bridge = CvBridge()
        j = 0
        for i, (_, msg, _) in enumerate(
                bag.read_messages(topics=[image_topic])):
            if i not in seq:
                continue
            try:
                image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                filename = '{0}/image_{1:03d}.png'.format(output_dir, j)
                # Write image to filename
                if cv2.imwrite(filename, image):
                    print('[{0}/{1}: {2}'.format(j, num_images, filename))
                    j += 1
            except CvBridgeError, e:
                print(e)


def main(argv):
    bagfile = '/home/chao/Workspace/bag/D512/exp_v1' \
              '/galt_experimental_v1_umbrella_led_row_forth_2015-07-31-01-31-02.bag'
    image_topic = '/color/image_raw'
    num_images = 20
    output_dir = '/tmp'

    try:
        extract_images(bagfile, output_dir, image_topic, num_images)
    except IOError, e:
        # invalid bag file
        print(e)
    except ValueError, e:
        # num_images > total_images
        # total_images == 0
        print(e)


if __name__ == '__main__':
    # TODO: add argparse
    # arguments:
    # input [bagfile]
    # output [directory]
    # image_topic [string]
    # num_images [int]
    # show_images [bool]
    sys.exit(main(sys.argv))
