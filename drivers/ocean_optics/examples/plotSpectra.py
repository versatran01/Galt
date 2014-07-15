#!/usr/bin/env python
# Subscribes to /spectrometer/spectrum topic and plots every 100th spectrum from the Ocean Optics Spectrum messages 
import rospy
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
plt.ion()


def callback(data):
	if (data.header.seq % 100) == 0:
		wavelengths = data.wavelengths	
		spectrum = data.spectrum
		plt.clf()
		plt.plot(wavelengths,spectrum,'-')
		plt.ylim([0,1000])
		plt.draw() 

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/spectrometer/spectrum", Spectrum, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
