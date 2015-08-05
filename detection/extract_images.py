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


def is_dir_exist_and_writable(dir):
    return os.path.isdir(dir) and os.path.exists(dir) \
        and os.access(dir, os.W_OK)


def extract_images(bagfile, output_dir, image_topic, num_images, visualize):
    print('Opening bag file: ', bagfile)
    with rosbag.Bag(bagfile, 'r') as bag:
        print('Done')
        total_images = bag.get_message_count(image_topic)

        if total_images == 0:
            raise ValueError('total_images == 0 under topic ', image_topic)

        if num_images > total_images:
            raise ValueError('num_images {0} > total_images {1}'.format(
                num_images, total_images))

        # seq encodes which image to sample out of all images
        seq = get_random_indices(num_images, total_images)

        # Go through all messages in this topic
        print('Extracting')
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
                    print('[{0}/{1}]: {2}'.format(j, num_images, filename))
                    j += 1
                    if visualize:
                        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
                        cv2.imshow('image', image)
                        cv2.waitKey(10)
            except CvBridgeError, e:
                print(e)


def main(argv):
    # Parse argument
    parser = argparse.ArgumentParser(
        description='Randomly extract images from bag file')
    parser.add_argument('input',
                        help='File name of the bag')
    parser.add_argument('--output_dir', '-o',
                        default=os.getcwd(),
                        help='Output directory of extracted images')
    parser.add_argument('--image_topic', '-t',
                        nargs='?',
                        default='/color/image_raw',
                        help='Topic name of image messages')
    parser.add_argument('--num_images', '-n',
                        nargs='?',
                        default=20,
                        type=int,
                        help='Number of images to extract')
    parser.add_argument('--visualize', '-v',
                        action='store_true',
                        help='Show images during extraction')
    args = parser.parse_args()
    print('You are going to extract [{0}] images under topic [{1}]'.format(
        args.num_images, args.image_topic))
    print('from bag file [{0}]'.format(args.input))
    print('into directory [{0}]'.format(args.output_dir))

    # Ask whether to proceed
    user_input = raw_input('Do you want to proceed: [y/n]')
    if user_input.lower() != 'y':
        sys.exit(1)

    # Extract images
    try:
        # Verify folder exist and we have writing access
        if not is_dir_exist_and_writable(args.output_dir):
            raise IOError('Ouptut dir does not exist or no write access.')
        extract_images(args.input, args.output_dir, args.image_topic,
                       args.num_images, args.visualize)
    except IOError, e:
        # invalid bag file
        # invalid output directory
        print(e)
    except ValueError, e:
        # num_images > total_images
        # total_images == 0
        print(e)


if __name__ == '__main__':
    # use -h to see arguments
    sys.exit(main(sys.argv))
