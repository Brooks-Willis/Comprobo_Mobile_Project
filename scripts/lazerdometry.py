#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class Lazer(object):
    def __init__(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        self.penultimate = None
        self.ultimate = None

    def scan_received(self, data):
        if data.ranges == self.ultimate:
            return
        self.penultimate = self.ultimate
        self.ultimate = data.ranges

    def get_old_scan(self):
        return list(self.penultimate)

    def get_new_scan(self):
        return list(self.ultimate)


class Odom(object):
    def __init__(self):
        self.sub = rospy.Subscriber('odom', Odometry, self.odom_received)
        self.old_odom = None
        self.new_odom = None
        self.deltas = np.array([[0], [0], [0]])  # dx, dy, dtheta

    def odom_received(self, data):
        self.old_odom = self.new_odom
        self.new_odom = data
        if self.old_odom != None:
            self.deltas[(0,0)] = (
                self.new_odom.pose.position.x - self.old_odom.pose.position.x
            )
            self.deltas[(1,0)] = (
                self.new_odom.pose.position.y - self.old_odom.pose.position.y
            )
            new_quat = self.new_odom.pose.orientation
            old_quat = self.old_odom.pose.orientation
            self.deltas[(2,0)] = new_quat.angle(old_quat)

    def get_deltas(self):
        return np.array(self.deltas)


class Lazerdom(object):
    def __init__(self):
        rospy.init_node('lazerdom', anonymous=True)
        self.pub = rospy.Publisher('corrected_odom', Pose, queue_size=20)
        self.lazer_sub = Lazer()
        self.odom_sub = rospy.Subscriber('odom', Odometry, odom_received)


if __name__ == '__main__':
    try:
        laughs = Odom()
    except rospy.ROSInterruptException:
        pass
