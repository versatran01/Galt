import rosbag
import os
import genpy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int32


def create_dir_if_not_exist(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def append_ext_to_file(filename, ext):
    if not filename.endswith(ext):
        filename += ext
    return filename


def append_ext_to_bag(bag_path):
    return append_ext_to_file(bag_path, '.bag')


def create_empty_bagfile(bag_path):
    bag_path = append_ext_to_bag(bag_path)
    with rosbag.Bag(bag_path, 'w') as bag:
        pass


def remove_bagfile(bag_path):
    os.remove(bag_path)


def create_fix_stamp_bagfile(bag_path, sync, topic='test', num_msgs=5):
    bag_path = append_ext_to_bag(bag_path)

    stamp_range = range(num_msgs)

    with rosbag.Bag(bag_path, 'w') as bag:
        for stamp in stamp_range:
            msg = Vector3Stamped()
            msg.header.stamp = genpy.Time(stamp)
            if sync:
                t = stamp
            else:
                t = stamp + 0.5
            t = genpy.Time(t)
            bag.write(topic, msg, t)


def create_headerless_bagfile(bag_path, headerless=True, num_msgs=5):
    bag_path = append_ext_to_bag(bag_path)

    with rosbag.Bag(bag_path, 'w') as bag:
        for stamp in xrange(num_msgs):
            msg = Vector3Stamped()
            msg.header.stamp = genpy.Time(stamp)
            t = genpy.Time(stamp)
            bag.write('has_header', msg, t)

        # write a headerless message
        if headerless:
            bag.write('headerless', Int32(), genpy.Time(num_msgs))
