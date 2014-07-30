import roslib 
import rospy

import hokuyo_node
from sensor_msgs.msg import LaserScan

def callback(data):
    print dir(data)


def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/hokuyo/scan",LaserScan,callback)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()
