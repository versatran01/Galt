#!/usr/bin/env python

import getopt
import sys
import os.path
import rosbag
from datetime import datetime
from rospy.rostime import Duration

class UbloxMessage():
    def __init__(self,topic,msg,t):
        self.topic = topic
        self.msg = msg
        self.t = t
        
        if topic=='/ublox_gps/ublox/navposllh':
            self.timeOfWeek = msg.iTOW
            # account for atomic offset
            self.timeOfWeek = self.timeOfWeek - 16000
            #print "navposllh %i" % self.timeOfWeek
        else:
            # /ublox_gps/fix
            stamp = msg.header.stamp
            
            # correct to UTC from EST
            stamp = stamp + Duration(4*60*60)
            
            secs = stamp.secs
            nsecs = stamp.nsecs
            
            # convert ROS time to date
            posix = secs + nsecs/1.0e9            
            D = datetime.fromtimestamp(posix)
            
            # generate time of week
            weekday = D.weekday()
            hour = D.hour
            minute = D.minute
            second = D.second
            milli = D.microsecond/1000;
            
            # GPS time starts from sunday
            weekday = (weekday + 1) % 6
            
            tow = (weekday*(3600*24) + hour*3600 + minute*60 + second)*1000 + milli
            
            # round to nearest 250ms to match ublox
            tow = int(tow / 250) * 250
            self.timeOfWeek = tow
            
class UbloxFixer():
    fixList = None
    navposList = None
    fixCount = 0
    navposCount = 0
    outputCount = 0
    
    def __init__(self):
        self.fixList = list()
        self.navposList = list()
        
    def considerMessage(self,topic,msg,t):
        if topic == '/ublox_gps/fix':
            self.fixList.append(UbloxMessage(topic,msg,t))
            self.fixCount = self.fixCount+1
        elif topic == '/ublox_gps/ublox/navposllh':
            self.navposList.append(UbloxMessage(topic,msg,t))
            self.navposCount = self.navposCount+1
            
        pubList = list()
            
        # check for matches in queue
        for fixMsg in self.fixList:
            for navposMsg in self.navposList:
                if fixMsg.timeOfWeek == navposMsg.timeOfWeek:
                    # insert tuple
                    pubList.append((fixMsg, navposMsg))
        
        # remove from the queues
        for pairs in pubList:
            self.fixList.remove(pairs[0])
            self.navposList.remove(pairs[1])
        
        # sort in chronometric order
        pubList = sorted(pubList, key = lambda pair: pair[0].timeOfWeek)
        
        # generate covariances and correct height
        convertedList = list()
        for pairs in pubList:
            # convert to meters
            hAcc = pairs[1].msg.hAcc / 1000.0
            vAcc = pairs[1].msg.vAcc / 1000.0
            hMSL = pairs[1].msg.hMSL / 1000.0
            
            # make a 'covariance' from the accuracies
            hAcc = (hAcc/3)*(hAcc/3)
            vAcc = (vAcc/3)*(vAcc/3)
            
            pairs[0].msg.altitude = hMSL
            pairs[0].msg.position_covariance = (hAcc,0.0,0.0,0.0,hAcc,0.0,0.0,0.0,vAcc)
            convertedList.append((pairs[0].topic, pairs[0].msg))
            self.outputCount = self.outputCount + 1
            
        return convertedList
        
def usage():
    print("Usage: fixbag --input=<input path> --output=<output path>")

def main(argv):
    inputPath = None
    outputPath = None

    fixer = UbloxFixer()

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
            fixer.considerMessage(topic, msg, t)
           
            # TODO: write out to bag file here...
           
main(sys.argv[1:])
