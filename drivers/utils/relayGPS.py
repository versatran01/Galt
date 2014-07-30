#!/usr/bin/env python
# Subscribes to /spectrometer/spectrum topic and plots every 100th spectrum from the Ocean Optics Spectrum messages 
import rospy
from ublox_msgs.msg import NavPOSLLH
from std_msgs.msg import String

pub = rospy.Publisher('gpsString', String)

def callback(data):
	iTOW = data.iTOW
	lon = (data.lon)*1e-7
	lat = (data.lat)*1e-7
	pub.publish(str(iTOW) + ','+ str(lat) + ',' + str(lon))

def listener():
	rospy.init_node('gpsNode', anonymous=True)
	rospy.Subscriber("/ublox_gps/ublox/navposllh", NavPOSLLH, callback)
	rospy.spin()
        
if __name__ == '__main__':
	listener()

