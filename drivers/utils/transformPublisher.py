#!/usr/bin/env python  
import roslib
import rospy
import tf
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavPOSLLH
import math

global lon	
global lat
global height 
global br

br = tf.TransformBroadcaster()
def callback(data):
	global lon 
	global lat
	global br
	do = data.orientation
	(roll,pitch,yaw) = tf.transformations.euler_from_quaternion((do.x, do.y,do.z,do.w))
	#13 45 57 E DECLINATION ANGLEi
	quat_corrIMUBASE = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
	quat_corrLASERBASE = tf.transformations.quaternion_from_euler(math.pi/2.0,0,math.pi/2.0)
	#br.sendTransform((0,0,0),(0,0,0,0),rospy.Time.now(),"/laser","/laser_base")
	br.sendTransform((lon, lat, height),(quat_corrIMUBASE),data.header.stamp,"/imu_base","/world")
	br.sendTransform((0, 0, 0),(quat_corrLASERBASE),data.header.stamp,"/laser","/imu_base")

def callbackGPS(data):
	global lon
	global lat
	global height
	iTOW = data.iTOW
	lon = ((-121.2429-(data.lon)*1e-7)*1e4)-69
	lat = ((38.3509-(data.lat)*1e-7)*1e4)+51
	height = ((data.hMSL)/1000.0)-17
def listener():
	rospy.init_node('imuNode', anonymous=True)
	rospy.Subscriber("/ublox_gps/ublox/navposllh", NavPOSLLH, callbackGPS)
	rospy.Subscriber("/attitude_eskf/filtered_imu", Imu, callback)
	rospy.spin()
        
if __name__ == '__main__':
	listener()

