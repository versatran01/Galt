#!/usr/bin/env python
# Subscribes to /spectrometer/spectrum topic and plots every 100th spectrum from the Ocean Optics Spectrum messages 
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import tf
import math
import matplotlib.pyplot as plt
plt.ion()


pub = rospy.Publisher('imuString', String)

def callback(data):
	do = data.orientation
	if(data.header.seq % 45==0):
		euler = tf.transformations.euler_from_quaternion((do.x,do.y,do.z, do.w))
		plt.plot(data.header.seq,euler[0],'ro')
		plt.plot(data.header.seq,euler[1],'bo')
		plt.plot(data.header.seq,euler[2],'ko')
	#print(math.degrees(euler[0]))
		plt.draw()

def listener():
	rospy.init_node('imuNode', anonymous=True)
	rospy.Subscriber("/attitude_eskf/filtered_imu", Imu, callback)
	rospy.spin()
        
if __name__ == '__main__':
	listener()






