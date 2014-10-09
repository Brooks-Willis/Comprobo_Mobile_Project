import unittest
import numpy as np
import tf
from lazerdometry import Lazer, Odom
from mock import MagicMock
from geometry_msgs.msg import Quaternion


class TestLazer(unittest.TestCase):
    def setUp(self):
        self.lazer = Lazer()

    def test_scan_received(self):
        scan1 = MagicMock()
        scan1.ranges.return_value = range(360)
        scan2 = MagicMock()
        scan2.ranges.return_value = range(1, 361)

        self.lazer.scan_received(scan1)
        self.assertEqual(self.lazer.penultimate, None)
        self.assertEqual(self.lazer.ultimate, scan1.ranges)

        self.lazer.scan_received(scan2)
        self.assertEqual(self.lazer.penultimate, scan1.ranges)
        self.assertEqual(self.lazer.ultimate, scan2.ranges)

        self.lazer.scan_received(scan2)
        self.assertEqual(self.lazer.penultimate, scan1.ranges)
        self.assertEqual(self.lazer.ultimate, scan2.ranges)

    def test_get_old_scan(self):
        range1 = range(360)
        self.lazer.penultimate = range1

        self.assertNotEqual(id(self.lazer.get_old_scan()), id(range1))
        self.assertEqual(self.lazer.get_old_scan(), range1)

    def test_get_new_scan(self):
        range1 = range(360)
        self.lazer.ultimate = range1

        self.assertNotEqual(id(self.lazer.get_new_scan()), id(range1))
        self.assertEqual(self.lazer.get_new_scan(), range1)


class TestOdom(unittest.TestCase):
    def setUp(self):
        self.odom = Odom()

	def make_quat(self, z, w):
		q = np.array([z, w])
		q = q/np.linalg.norm(q)

		quat = Quaternion(x=0, y=0, z=q[0], w=q[1])

		return quat

	def test_quat_distance(self):
		odom1 = MagicMock()
		odom1.pose.position.x.return_value = 0
		odom1.pose.position.y.return_value = 0
		odom1.pose.orientation.x.return_value = self.make_quat(0,1)

		odom2 = MagicMock()
		odom2.pose.position.x.return_value = 0
		odom2.pose.position.y.return_value = 0
		odom2.pose.orientation.x.return_value = self.make_quat(1,1)

		self.odom.odom_received(odom1)

		self.assertEqual(self.odom.old_odom,None)
		self.assertEqual(self.odom.new_odom,odom1)
		self.assertTrue(np.array_equal(self.odom.deltas,np.array([[0],[0],[0]])))

		self.odom.odom_received(odom2)

		self.assertEqual(self.odom.old_odom,odom1)
		self.assertEqual(self.odom.new_odom,odom2)
		temp = self.odom.old_odom.pose.position.x
		print temp
		print self.odom.new_odom.pose.position.x
		print self.odom.deltas
		self.assertTrue(np.array_equal(self.odom.deltas,np.array([[0],[0],[90]])))

    def test_quat_distance(self):
        q1 = self.make_quat(0, 1)
        q2 = self.make_quat(1, 1)
        self.assertEqual(self.odom.quat_distance(q2, q1), np.degrees(1))
