from __future__ import absolute_import, print_function, unicode_literals
import unittest2 as unittest
from .create_bagfiles import *
import bag_tools.bag_tools as bt

g_test_home = '/tmp/test_bag_tools'
create_dir_if_not_exist(g_test_home)


class TestBagHelperStamp(unittest.TestCase):
    def setUp(self):
        # This is ugly, fix later
        headerless_bag_path = g_test_home + '/headerless.bag'
        all_header_bag_path = g_test_home + '/all_header.bag'
        create_headerless_bagfile(headerless_bag_path, headerless=True)
        create_headerless_bagfile(all_header_bag_path, headerless=False)
        self.headerless_bag_helper = bt.BagHelper(headerless_bag_path)
        self.all_header_bag_helper = bt.BagHelper(all_header_bag_path)

        stamp_fixed_bag_path = g_test_home + '/stamp_fixed.bag'
        stamp_not_fixed_bag_path = g_test_home + '/stamp_not_fixed.bag'
        create_fix_stamp_bagfile(stamp_fixed_bag_path, sync=True)
        create_fix_stamp_bagfile(stamp_not_fixed_bag_path, sync=False)
        self.stamp_fixed_bag_helper = bt.BagHelper(stamp_fixed_bag_path)
        self.stamp_not_fixed_bag_helper = bt.BagHelper(stamp_not_fixed_bag_path)

    def test_has_headerless_msg(self):
        self.assertTrue(self.headerless_bag_helper.has_headerless_msg())
        self.assertFalse(self.all_header_bag_helper.has_headerless_msg())

    def test_is_timestamp_fixed(self):
        self.assertTrue(self.stamp_fixed_bag_helper.is_timestamp_fixed())
        self.assertFalse(self.stamp_not_fixed_bag_helper.is_timestamp_fixed())

    def test_fix_timestamp(self):
        self.stamp_not_fixed_bag_helper.fix_timestamp()
        stamp_fixed_bag_helper = bt.BagHelper(
            self.stamp_not_fixed_bag_helper.out_bag_path)
        self.assertTrue(stamp_fixed_bag_helper.is_timestamp_fixed())
        remove_bagfile(stamp_fixed_bag_helper.in_bag_path)

    def tearDown(self):
        remove_bagfile(self.headerless_bag_helper.in_bag_path)
        remove_bagfile(self.all_header_bag_helper.in_bag_path)
        remove_bagfile(self.stamp_fixed_bag_helper.in_bag_path)
        remove_bagfile(self.stamp_not_fixed_bag_helper.in_bag_path)


class TestBagBatchProcessorBase(unittest.TestCase):
    def setUp(self):
        self.num_bags = 3
        self.bags_to_create = []
        for i in xrange(self.num_bags):
            bag_path = append_ext_to_bag(g_test_home + '/bag' + str(i))
            self.bags_to_create.append(bag_path)

        for bag in self.bags_to_create:
            create_empty_bagfile(bag)

        parser = bt.get_io_parser()
        args = parser.parse_args([g_test_home])

        self.batch_proc = bt.BagBatchProcessorBase(args)

    def test_process_args(self):
        self.batch_proc.process_args()
        self.assertEqual(len(self.batch_proc.all_bag_helpers), self.num_bags)
        for bag_helper in self.batch_proc.all_bag_helpers:
            self.assertEqual(bag_helper.out_dir, g_test_home)
            self.assertEqual(bag_helper.bag_ext, '.bag')

    def tearDown(self):
        for bag in self.bags_to_create:
            remove_bagfile(bag)
