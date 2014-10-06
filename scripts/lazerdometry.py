#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class Lazerdom():
    def __init__(self):
        self.pub = rospy.Publisher('corrected_odom', Pose, queue_size=10)
        self.lazer_sub = rospy.Subscriber('scan', LaserScan, scan_received)
        self.odom_sub = rospy.Subscriber('odom',Odometry, odom_received)
        rospy.init_node('lazerdom',anonymous=True)

    def scan_received(self,data):
        




        
if __name__ == '__main__':
    try:
        
    except rospy.ROSInterruptException: pass
