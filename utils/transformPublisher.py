#!/usr/bin/env python  
import roslib
import rospy
import tf
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPOSLLH
import math
import utm 
global northing
global easting
global height 
global br

#In [3]: utm.from_latlon(38.351105, -121.249039)
#Out[3]: (652999.4014047047, 4246222.846376365, 10, 'S')


br = tf.TransformBroadcaster()
def callback(data):
	global northing 
	global easting
	global br
	global height
	do = data.orientation
	(roll,pitch,yaw) = tf.transformations.euler_from_quaternion((do.x, do.y,do.z,do.w))
	#13 45 57 E DECLINATION ANGLEi
	quat_corrIMUBASE = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
	#below: direction of scan seems OK, but fails for road walk 
	#quat_corrLASERBASE = tf.transformations.quaternion_from_euler(0,-math.pi,-math.pi/2.0)
	
	#below: tranform that works on steadicam setup 
	quat_corrLASERBASE = tf.transformations.quaternion_from_euler(math.pi,0,math.pi/2.0)

	#br.sendTransform((0,0,0),(0,0,0,0),rospy.Time.now(),"/laser","/laser_base")
	br.sendTransform((easting, northing, height),(quat_corrIMUBASE),data.header.stamp,"/imu_base","/world")
	br.sendTransform((0, 0, 0),(quat_corrLASERBASE),data.header.stamp,"/laser","/imu_base")

def callbackGPS(data):
	global northing
	global easting
	global height
	(eastingOffset, northingOffset, zoneNumber, zoneLetter) = utm.from_latlon(38.355972, -121.249039)
	lonGPS = (data.lon)*1e-7
	latGPS = (data.lat)*1e-7
	(easting,northing ,zoneNumber ,zoneLetter) = utm.from_latlon(latGPS, lonGPS)
	northing = northing - northingOffset  
	easting = easting - eastingOffset
	height = ((data.hMSL)/1000.0)-15
def listener():
	rospy.init_node('imuNode', anonymous=True)
	rospy.Subscriber("/ublox_gps/ublox/navposllh", NavPOSLLH, callbackGPS)
	rospy.Subscriber("/attitude_eskf/filtered_imu", Imu, callback)
	rospy.spin()
        
if __name__ == '__main__':
	listener()

