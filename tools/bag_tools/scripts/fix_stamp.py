#!/usr/bin/python

import bag_tools.bag_tools as bt
import sys
import argparse as ap


def main():
    parser = ap.ArgumentParser(description='Fix timestamp of bag files.',
                               parents=[bt.get_io_parser(),
                                        bt.get_common_options_parser()])
    parser.add_argument('-s', '--skip-headerless', action='store_true',
                        help='Skip bag files that have headerless messsages.')

    args = parser.parse_args()
    print(args)
    bag_stamp_batch_proc = bt.BagStampBatchProcessor(args)
    bag_stamp_batch_proc.process_args()
    bag_stamp_batch_proc.process_bags()


if __name__ == '__main__':
    sys.exit(main())
