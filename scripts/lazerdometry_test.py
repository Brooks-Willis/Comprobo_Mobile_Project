import unittest
import numpy as np
import tf
from lazerdometry import Lazer, Odom, Lazerdom
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

    def test_odom_received(self):
        odom1 = MagicMock()
        odom1.pose.position.x = 0
        odom1.pose.position.y = 0
        odom1.pose.orientation.x = self.make_quat(0,1)

        odom2 = MagicMock()
        odom2.pose.position.x = 0
        odom2.pose.position.y = 0
        odom2.pose.orientation.x = self.make_quat(1,1)

        self.odom.odom_received(odom1)

        self.assertEqual(self.odom.old_odom,odom1)
        self.assertEqual(self.odom.new_odom,odom1)

        self.odom.odom_received(odom2)

        self.assertEqual(self.odom.old_odom,odom1)
        self.assertEqual(self.odom.new_odom,odom2)

    def test_deltas(self):
        odom1 = MagicMock()
        odom1.pose.position.x = 0
        odom1.pose.position.y = 0
        odom1.pose.orientation = self.make_quat(0,1)

        odom2 = MagicMock()
        odom2.pose.position.x = 0
        odom2.pose.position.y = 0
        odom2.pose.orientation = self.make_quat(1,1)

        self.odom.old_odom = odom1
        self.odom.new_odom = odom1

        self.assertTrue(np.array_equal(self.odom.deltas(),np.array([[0],[0],[0]])))

        self.odom.new_odom = odom2

        self.assertTrue(np.array_equal(self.odom.deltas(),np.array([[0],[0],[90]])))

    def test_get_deltas(self):
        odom1 = MagicMock()
        odom1.pose.position.x = 0
        odom1.pose.position.y = 0
        odom1.pose.orientation = self.make_quat(0,1)

        odom2 = MagicMock()
        odom2.pose.position.x = 0
        odom2.pose.position.y = 0
        odom2.pose.orientation = self.make_quat(1,1)

        self.odom.new_odom = odom2
        self.odom.old_odom = odom1

        dels = self.odom.get_deltas()

        self.assertEqual(self.odom.old_odom,odom2)
        self.assertTrue(np.array_equal(dels,np.array([[0],[0],[90]])))


class TestLazerdom(unittest.TestCase):
    def setUp(self):
        self.lazerdom = Lazerdom()

    def make_quat(self, z, w):
        q = np.array([z, w])
        q = q/np.linalg.norm(q)

        quat = (0, 0, q[0], q[1])

        return quat

    def test_compute_new_pose(self):
        actual_del = np.array([[1],[2],[90]])

        self.lazerdom.compute_new_pose(actual_del)

        self.assertTrue(np.allclose(self.lazerdom.current_quat,self.make_quat(1,1)))
        self.assertEqual(self.lazerdom.current_pos,[1,2,0])

        actual_del = np.array([[1],[2],[-90]])

        self.lazerdom.compute_new_pose(actual_del)

        self.assertTrue(np.allclose(self.lazerdom.current_quat,self.make_quat(0,1)))
        self.assertEqual(self.lazerdom.current_pos,[2,4,0])