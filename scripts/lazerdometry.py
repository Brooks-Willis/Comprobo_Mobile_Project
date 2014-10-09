#!/usr/bin/env python

import rospy
import numpy as np
import tf
import gradient_descent as grd
import residuals as rsd
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion


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
        if self.penultimate != None:
            return list(self.penultimate)
        return None

    def get_new_scan(self):
        if self.ultimate != None:
            return list(self.ultimate)
        return None


class Odom(object):
    def __init__(self):
        self.sub = rospy.Subscriber('odom', Odometry, self.odom_received)
        self.old_odom = None
        self.new_odom = None
          # dx, dy, dtheta

    def deltas(self):
        if not (self.old_odom and self.new_odom):
            return None

        out = np.array([[0], [0], [0]])

        out[0,0] += (
            self.new_odom.pose.pose.position.x - self.old_odom.pose.pose.position.x
        )
        out[1,0] += (
            self.new_odom.pose.pose.position.y - self.old_odom.pose.pose.position.y
        )
        new_quat = (self.new_odom.pose.pose.orientation.x,
            self.new_odom.pose.pose.orientation.y,
            self.new_odom.pose.pose.orientation.z,
            self.new_odom.pose.pose.orientation.w)
        old_quat = (self.old_odom.pose.pose.orientation.x,
            self.old_odom.pose.pose.orientation.y,
            self.old_odom.pose.pose.orientation.z,
            self.old_odom.pose.pose.orientation.w)
        new_yaw = tf.transformations.euler_from_quaternion(new_quat)[2]
        old_yaw = tf.transformations.euler_from_quaternion(old_quat)[2]
        out[2,0] += np.degrees(new_yaw-old_yaw)
        return out

    def odom_received(self, data):
        if self.old_odom == None:
            self.old_odom = data
        self.new_odom = data

    def get_deltas(self):
        temp = self.deltas()
        self.old_odom = self.new_odom
        return temp


class Lazerdom(object):
    def __init__(self):
        rospy.init_node('lazerdom', anonymous=True)
        self.pub = rospy.Publisher('corrected_odom', Pose, queue_size=20)
        self.lazer_sub = Lazer()
        self.odom_sub = Odom()
        self.current_quat = (0,0,0,1)
        self.current_pos = [0,0,0]
        self.pose = None

    def compute_new_pose(self, actual_del):
        dx = actual_del[0,0]
        dy = actual_del[1,0]
        dtheta = actual_del[2,0]

        self.current_pos[0] += dx
        self.current_pos[1] += dy

        quat = tf.transformations.quaternion_from_euler(0,0,np.radians(dtheta))
        quat /= np.linalg.norm(quat)
        self.current_quat = tf.transformations.quaternion_multiply(
            quat,
            self.current_quat)
        self.current_quat /= np.linalg.norm(self.current_quat)

        position = Point(x=self.current_pos[0],
            y=self.current_pos[1],
            z=self.current_pos[2])

        orientation = Quaternion(x = self.current_quat[0],
            y = self.current_quat[1],
            z = self.current_quat[2],
            w = self.current_quat[3])

        self.pose = Pose(position = position, orientation = orientation)

    def publish_pose(self):
        self.pub.publish(self.pose)


    def execute(self):
        # run awwwwwwwwwwwwwwway
        r = rospy.Rate(5)
        while not(rospy.is_shutdown()):

            old = self.lazer_sub.get_old_scan()
            new = self.lazer_sub.get_new_scan()
            guess_del = self.odom_sub.get_deltas()

            if old != None and new != None and guess_del != None:
                res_func = lambda vect: rsd.residual(old,new,vect[0,0],vect[1,0],vect[2,0])

                actual_del = grd.gradient_descent(res_func,guess_del)

                self.compute_new_pose(actual_del)
                self.publish_pose()
            r.sleep()



if __name__ == '__main__':
    try:
        lazerdom = Lazerdom()
        lazerdom.execute()
    except rospy.ROSInterruptException:
        pass
