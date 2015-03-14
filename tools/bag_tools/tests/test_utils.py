from __future__ import absolute_import, print_function
import unittest
import bag_tools.utils as btu


class TestParser(unittest.TestCase):
    def setUp(self):
        self.input_args = ['in_dir1', 'in_dir2', 'file1.ext']
        self.output_args = ['-o', 'out_dir']
        self.io_args = self.input_args + self.output_args
        self.co_args = ['-f', '-q']

        self.io_parser = btu.get_io_parser()
        self.co_parser = btu.get_common_options_parser()

    def test_io_parser_input_only(self):
        args = self.io_parser.parse_args(self.input_args)
        self.assertListEqual(args.input, self.input_args)

    def test_io_parser_input_output(self):
        args = self.io_parser.parse_args(self.io_args)
        self.assertEqual(args.output_dir, 'out_dir')

    def test_co_parser(self):
        args = self.co_parser.parse_args(self.co_args)
        self.assertTrue(args.force_yes)
        self.assertTrue(args.quiet)
