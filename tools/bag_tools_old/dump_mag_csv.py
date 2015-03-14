#!/usr/bin/env python
# Tool for dumping magnetometer data to CSV.

import getopt
import sys
import os.path
import rosbag

def usage():
    print("Usage: dump_mag_csv.py --input=<input path> --output=<output path> [--topic=/imu/imu]")

def main(argv):
    inputPath = None
    outputPath = None
    topicName = '/imu/magnetic_field'

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
        elif opt == "--topic":
            topicName = arg

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
    with open(outputPath, 'w') as outputFile:
        for topic, msg, t in rosbag.Bag(inputPath).read_messages():
            if topic == topicName:
                x = msg.magnetic_field.x
                y = msg.magnetic_field.y
                z = msg.magnetic_field.z
                outputFile.write("{0},{1},{2}\n".format(x,y,z))

main(sys.argv[1:])
