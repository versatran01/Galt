#!/usr/bin/env python2

from __future__ import print_function
import rosbag
from nav_msgs.msg import Odometry

bagfile = '/home/chao/Workspace/bag/new_odom.bag'
odom_topic = '/gps_kf/odometry'
txtfile = '/home/chao/Desktop/new_poses.txt'

with rosbag.Bag(bagfile, 'r') as bag:
    print('open bag')
    with open(txtfile, 'w') as f:
        header = 'time, px, py, pz, qw, qx, qy, qz\n'
        f.write(header)

        for i, (topic, msg, t) in enumerate(
                bag.read_messages(topics=[odom_topic])):
            if (i % 10) == 0:
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                line = '{0}.{1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}\n'.format(
                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                    position.x, position.y, position.z, orientation.w,
                    orientation.x, orientation.y, orientation.z)
                f.write(line)
