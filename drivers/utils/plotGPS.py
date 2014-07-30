#!/usr/bin/env python
# Subscribes to /spectrometer/spectrum topic and plots every 100th spectrum from the Ocean Optics Spectrum messages 
import rospy
from ublox_msgs.msg import NavPOSLLH
import matplotlib.pyplot as plt
plt.ion()


def callback(data):
	lon = (data.lon)*1e-7
	lat = (data.lat)*1e-7
	plt.plot(lon,lat,'o')
	39.9527507,-75.1921067
	39.9514102,-75.1905201
	#plt.xlim([-75.1921067,-75.1905201])
	#plt.ylim([39.9514102,39.9527507])
	plt.draw() 

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/ublox/navposllh", NavPOSLLH, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()

