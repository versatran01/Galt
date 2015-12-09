#!/usr/bin/env python2
import rosbag
from geometry_msgs.msg import PoseStamped

bagfile = '/media/chao/Seagate/galt-2015-10-17-22-29-22.bag'
pose_topic = '/gps_kf/pose'
txtfile = '/home/chao/Desktop/new_poses.txt'

with rosbag.Bag(bagfile, 'r') as bag:
    print('open bag')
    with open(txtfile, 'w') as f:
        header = 'time, px, py, pz, qw, qx, qy, qz\n'
        f.write(header)

        for i, (topic, msg, t) in enumerate(
                bag.read_messages(topics=[pose_topic])):
            if (i % 1) == 0:
                position = msg.pose.position
                orientation = msg.pose.orientation
                line = '{0}.{1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}\n'.format(
                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                    position.x, position.y, position.z, orientation.w,
                    orientation.x, orientation.y, orientation.z)
                f.write(line)
