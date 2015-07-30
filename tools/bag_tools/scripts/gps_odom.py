#!/usr/bin/env python
import rospy
import sys
import message_filters
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import utm


class gps_odom:

    def __init__(self):
        self.fix_sub = message_filters.Subscriber('gps/fix', NavSatFix)
        self.twist_sub = message_filters.Subscriber('gps/fix_velocity', TwistWithCovarianceStamped)
        self.odom_pub = rospy.Publisher('gps/odom', Odometry, queue_size=10)
        self.fix_pub = rospy.Publisher('gps/fix_fake', NavSatFix, latch=True, queue_size=1)

        self.ts = message_filters.TimeSynchronizer([self.fix_sub, self.twist_sub], 10)
        self.ts.registerCallback(self.synced_callback)

        self.received = False
        self.initial_x = 0
        self.initial_y = 0
        self.initial_z = 0

    def synced_callback(self, fix, twist):

        odom = Odometry()

        odom.header = twist.header
        odom.header.frame_id = 'world'
        odom.twist = twist.twist

        odom.pose.pose.position.z = 0.8

        # convert to UTM
        utm_coordinates = utm.from_latlon(fix.latitude, fix.longitude)
        odom.pose.pose.position.x = utm_coordinates[0] - self.initial_x
        odom.pose.pose.position.y = utm_coordinates[1] - self.initial_y

        if not self.received:
            odom.pose.pose.position.x = 0
            odom.pose.pose.position.y = 0
            odom.pose.pose.position.z = 0.8
            self.initial_x = utm_coordinates[0]
            self.initial_y = utm_coordinates[1]
            self.initial_z = fix.altitude
            self.received = True
            self.fix_pub.publish(fix)

        self.odom_pub.publish(odom)


def main(args):
    gps = gps_odom()
    rospy.init_node('gps_odom', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
