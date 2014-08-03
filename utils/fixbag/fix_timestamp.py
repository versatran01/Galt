#!/usr/bin/env python

import getopt
import sys
import os.path
import rosbag


def usage():
    print("Usage: fixbag --input=<input path> --output=<output path>")


def main(argv):
    inputPath = None
    outputPath = None

    # parse input arguments
    try:
        opts, args = getopt.getopt(argv, "h", ['input=', 'output=', 'help'])
    except getopt.GetoptError:
        usage()
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage()
            sys.exit()
        elif opt == "--input":
            inputPath = arg
        elif opt == "--output":
            outputPath = arg

    if inputPath == None or outputPath == None:
        usage()
        sys.exit(3)

    inputPath = os.path.normpath(os.path.expanduser(inputPath))
    outputPath = os.path.normpath(os.path.expanduser(outputPath))

    # check file paths
    if not os.path.isfile(inputPath):
        print "Input path %s is invalid" % (inputPath)
        sys.exit(4)

    # from the ros wiki:
    with rosbag.Bag(outputPath, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inputPath).read_messages():
            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            if topic == "/tf" and msg.transforms:
                outbag.write(topic, msg, msg.transforms[0].header.stamp)
            else:
                outbag.write(
                    topic, msg, msg.header.stamp if msg._has_header else t)

main(sys.argv[1:])
