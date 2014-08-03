#!/usr/bin/env python

from __future__ import print_function
import sys
import os.path
import time
import timeit
import fixbag


def get_all_bagfiles(input_path):
    all_bagfiles = []

    if not os.path.isdir(input_path):
        print('input and output must be folder')
        sys.exit(1)

    print("Scanning all bag files in", input_path)
    for root, dirs, files in os.walk(input_path):
        # Get all bag files under current directory
        bagfiles = [bagfile for bagfile in files if bagfile.endswith('.bag')]
        print('Scanning {0:s} ... {1:d} bag files'.format(root, len(bagfiles)))
        # Get size of bag files
        if bagfiles:
            all_bagfiles.extend([os.path.join(root, bagfile) for bagfile in bagfiles])

    return all_bagfiles


def get_total_size(bagfiles):
    total_size_in_gb = 0
    for index, bagfile in enumerate(bagfiles):
        bag_size_in_gb = os.path.getsize(bagfile)/(1024**3)
        total_size_in_gb += bag_size_in_gb
        print("{0:d}: {1:s} - {2:.1f}G".format(index, bagfile, bag_size_in_gb))

    return total_size_in_gb


def query_yes_or_no():
    choice = raw_input("Do you wish to fix all the bag files listed above? [y/n]").lower()
    if choice in ('y', 'yes'):
        print("Start fixing bagfiles, this will probably take hours.")
    else:
        print("Abort.")
        sys.exit(1)


def process_in_and_out(input_path, output_path):
    input_path = os.path.normpath(os.path.expanduser(input_path))
    output_path = os.path.normpath(os.path.expanduser(output_path))
    return input_path, output_path


def main():
    input_path = '~/Desktop/test_in'
    output_path = '~/Desktop/test_out'
    (input_path, output_path) = process_in_and_out(input_path, output_path)
    print("input path:", input_path)
    print("output path:", output_path)

    all_bagfiles = get_all_bagfiles(input_path)

    print("All bagfiles to be fixed:")
    print("Total size: {0:.1f}G".format(get_total_size(all_bagfiles)))

    query_yes_or_no()

    total_time = 0
    # Fixing bagfiles
    for index, bagfile in enumerate(all_bagfiles):
        # Print progress
        progress = '{0:d}/{1:d}'.format(index + 1, len(all_bagfiles))
        print('{0:s}: Start fixing {1:s}'.format(progress, bagfile))

        start_time = timeit.default_timer()

        # Fix bagfile
        fixbag.fix_bag(bagfile, input_path, output_path)

        elapsed_time = timeit.default_timer() - start_time
        total_time += elapsed_time
        print('Time:', elapsed_time)

    print("Total time:", time.strftime('%H:%M:%S', time.gmtime(elapsed_time)))


if __name__ == '__main__':
    sys.exit(main())
