#!/usr/bin/env python  
import roslib
import rospy
import tf
import math
import utm
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan	 
#pub = rospy.Publisher('laserRepublisherKF', LaserScan)

global quat_corrLASERBASE 
quat_corrLASERBASE = tf.transformations.quaternion_from_euler(math.pi/2.0,math.pi/2.0,0)


#In [3]: utm.from_latlon(38.351105, -121.249039)
#Out[3]: (652999.4014047047, 4246222.846376365, 10, 'S')

#global laserData

br = tf.TransformBroadcaster()

def callbackLaser(data):
	global laserData 
	laserData = data

	
def callback(data):
#	global laserData
	global quat_corrLASERBASE
	do = data.pose.pose.orientation
	dp = data.pose.pose.position
	(roll,pitch,yaw) = tf.transformations.euler_from_quaternion((do.x, do.y,do.z,do.w))
	#13 45 57 E DECLINATION ANGLEi
	quat_corrIMUBASE = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
	#below: direction of scan seems OK, but fails for road walk 
	#quat_corrLASERBASE = tf.transformations.quaternion_from_euler(math.pi/2.0,math.pi/2.0,0)
	
	#below: tranform that works on steadicam setup 
	#quat_corrLASERBASE = tf.transformations.quaternion_from_euler(math.pi/2,0,math.pi/2.0)
	#br.sendTransform((0,0,0),(0,0,0,0),rospy.Time.now(),"/laser","/laser_base")
	br.sendTransform((0, 0, 0),(quat_corrLASERBASE),data.header.stamp,"/laser","/imu_base")

	br.sendTransform((dp.x, dp.y, dp.z),(quat_corrIMUBASE),data.header.stamp,"/imu_base","/world")
	#pub.publish(laserData)
	#print data.header.stamp

def listener():
	rospy.init_node('kfTransformPublisherNode', anonymous=False)
	rospy.Subscriber("/gps_kf/odometry", Odometry, callback)
	#rospy.Subscriber("/hokuyo/scan", LaserScan, callbackLaser)
	rospy.spin()
        
if __name__ == '__main__':
	listener()

