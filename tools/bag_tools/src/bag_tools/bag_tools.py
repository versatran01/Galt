from __future__ import absolute_import, print_function, division

from .utils import *
import rosbag
from rosbag.rosbag_main import ProgressMeter


class BagHelper:
    def __init__(self, in_bag_path, out_dir=None, suffix="fixed"):
        # in and out should be valid full path
        self.in_bag_path = in_bag_path
        self.out_dir = out_dir

        self.bag_file = os.path.basename(self.in_bag_path)
        self.bag_name, self.bag_ext = os.path.splitext(self.bag_file)

        in_dir = os.path.dirname(self.in_bag_path)
        if self.out_dir is None or os.path.samefile(self.out_dir, in_dir):
            self.out_dir = in_dir

        self.out_bag_path = os.path.join(
            self.out_dir,
            self.bag_name + "_" + suffix + self.bag_ext)

    def __str__(self):
        return "input:      ${0:s}\n" \
               "output:     ${1:s}\n" \
               "output_dir: ${2:s}".format(self.in_bag_path, self.out_bag_path,
                                           self.out_dir)

    def has_headerless_msg(self):
        """
        :return: True if bag has headerless message
        """
        with rosbag.Bag(self.in_bag_path) as bag:
            for topic, msg, t in bag.read_messages():
                if not msg._has_header:
                    return True
            return False

    def is_timestamp_fixed(self):
        """
        :return: True if bag timestamp is already fixed
        """
        with rosbag.Bag(self.in_bag_path) as bag:
            for topic, msg, t in bag.read_messages():
                if msg.header.stamp != t:
                    return False
            return True

    def fix_timestamp(self, enable_meter=True):
        with rosbag.Bag(self.in_bag_path) as in_bag:
            with rosbag.Bag(self.out_bag_path, 'w') as out_bag:
                # To display progress meter, we need raw message
                if enable_meter:
                    total_bytes = 0
                    meter = ProgressMeter(self.bag_name, in_bag.size)
                    for topic, raw_msg, t in in_bag.read_messages(raw=True):
                        msg_type, serialized_bytes, md5sum, pos, pytype = raw_msg
                        msg = pytype()
                        msg.deserialize(serialized_bytes)

                        out_bag.write(topic, msg,
                                      msg.header.stamp if msg._has_header else t)

                        total_bytes += len(serialized_bytes)
                        meter.step(total_bytes)

                    meter.finish()
                else:
                    for topic, msg, t in in_bag.read_messages():
                        out_bag.write(topic, msg,
                                      msg.header.stamp if msg._has_header else t)


def process_input_args(in_args, recursive=False):
    all_inputs = []

    for in_arg in in_args:
        if os.path.isdir(in_arg):
            all_inputs.extend(get_all_bagfiles_in_dir(in_arg, recursive))
        elif os.path.isfile(in_arg):
            if in_arg.endswith('.bag'):
                all_inputs.append(in_arg)

    return all_inputs


def process_io_args(args):
    all_in_bags = process_input_args(args.input, args.recursive)

    # If output_dir is specified, make sure it exists
    if args.output_dir is not None:
        make_sure_path_exists(args.output_dir)

    # Create BagHelper for further processing
    bag_helpers = []
    for in_bag in all_in_bags:
        bag_helpers.append(BagHelper(in_bag, args.output_dir))

    return bag_helpers


class BagBatchProcessorBase:
    def __init__(self, args):
        self.args = args
        self.all_bag_helpers = None

    def process_args(self):
        self.all_bag_helpers = process_io_args(self.args)

    def process_bags(self):
        print('ProcessorBase process_bags method. Should be overwritten')


class BagStampBatchProcessor(BagBatchProcessorBase):
    def __init__(self, args):
        BagBatchProcessorBase.__init__(self, args)

    def process_args(self):
        print('Processing arguments...')
        BagBatchProcessorBase.process_args(self)

    def process_bags(self):
        # Do nothing if there's no bag files to process
        if not self.all_bag_helpers:
            print('No bags to process.')
            return

        # Process each bag files
        for bag_helper in self.all_bag_helpers:
            # skip headerless will skip processing bag file that has headerless
            # message
            print('Check if {0} has headerless message'.format(
                bag_helper.bag_file))
            if bag_helper.has_headerless_msg():
                print('Skipping {0} since it has headerless message'.format(
                    bag_helper.bag_file))
                continue

            # Move fixed will move fixed bag file to output dir instead of
            # copying it
            print('Check if {0} has correct timestamps'.format(
                bag_helper.bag_file))
            if bag_helper.is_timestamp_fixed():
                if self.args.move_fixed:
                    print('{0} will be moved.'.format(bag_helper.bag_file))
                else:
                    print('{0} will be copied.'.format(bag_helper.bag_file))

                continue

            # Fix bag timestamp
            bag_helper.fix_timestamp()
