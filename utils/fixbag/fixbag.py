#!/usr/bin/env python

from __future__ import print_function
import os
import os.path
import rosbag
import sys
import camera_info_manager
import fix_ublox


def read_left_cinfo():
    left_cinfo_manager = camera_info_manager.CameraInfoManager(
        'mv_25000855', 'package://galt_setup/calib/calib_25000855.yml')
    left_cinfo_manager.loadCameraInfo()
    return left_cinfo_manager.getCameraInfo()


fixer = fix_ublox.UbloxFixer()
left_cinfo = read_left_cinfo()
left_image_name = '/mv_stereo/left/image_raw'
left_cinfo_name = '/mv_stereo/left/camera_info'
flir_image_name = '/flir_gige/image_raw'
flir_cinfo_name = '/flir_gige/camera_info'


def fix_bag(input_bag, input_path, output_path):
    relative_path = os.path.relpath(input_bag, input_path)
    output_bag = os.path.join(output_path, relative_path)
    print("Fixed bag file will be written to", output_bag)

    # Create the directory if it does not exist
    output_bag_dir = os.path.dirname(output_bag)
    if not os.path.exists(output_bag_dir):
        os.makedirs(output_bag_dir)

    flir_cinfo = None
    # Open the bag file
    with rosbag.Bag(output_bag, 'w') as outbag:
        for index, (topic, msg, t) in enumerate(rosbag.Bag(input_bag).read_messages()):
            # Print progress
            if not (index % 5000):
                print(index, end=" ")
                sys.stdout.flush()

            if topic == left_image_name:
                left_cinfo.header = msg.header
                # Write left image to outbag
                outbag.write(topic, msg, msg.header.stamp)
                # Write left cinfo to outbag
                outbag.write(left_cinfo_name, left_cinfo, left_cinfo.header.stamp)
            elif topic == flir_cinfo_name:
                # Get and save the first flir camera info
                if not flir_cinfo:
                    flir_cinfo = msg
            elif topic == flir_image_name:
                # Write flir camera info with flir image
                if flir_cinfo:
                    flir_cinfo.header = msg.header
                    outbag.write(topic, msg, msg.header.stamp)
                    outbag.write(flir_cinfo_name, flir_cinfo, flir_cinfo.header.stamp)
            else:
                gps_msg_list = fixer.consider_message(topic, msg, t)
                print(gps_msg_list)
                if topic != '/ublox_gps/fix':
                    # Write the rest of the topic
                    outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
                else:
                    for (gps_topic, gps_msg) in gps_msg_list:
                        outbag.write(gps_topic, gps_msg, gps_msg.header.stamp)





