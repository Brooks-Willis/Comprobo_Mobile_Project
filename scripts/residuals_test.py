import unittest
import numpy as np
from residuals import makeFunc, wrapInterp, interpaLazers, computeResidual, translate, cartesian, rotate_cart, residual


class TestInterpolation(unittest.TestCase):
    def test_makeFunc(self):
        func = makeFunc(1, 2)
        self.assertEqual(func(0.3), 1.3)

    def dummyFunc1(self, x):
    	return x+1

    def dummyFunc2(self, x):
    	return x+4

    def test_wrapInterp(self):
    	interpArray=[self.dummyFunc1, self.dummyFunc2]
    	self.assertEqual(wrapInterp(0.3, interpArray), 1.3)
    	self.assertEqual(wrapInterp(1.3, interpArray), 4.3)
        self.assertEqual(wrapInterp(3.3, interpArray),4.3)

    def test_interpaLazers(self):
        interp = interpaLazers(range(360))
        self.assertEqual(interp(359.5), 179.5)
        self.assertEqual(interp(182.2365),182.2365)

    def test_computeResidual(self):
        old = range(360)
        new = [o+.5 for o in old]
        new_thetas = list(new)
        new[-1] = 179.5
        self.assertEqual(computeResidual(old,new,new_thetas),0)
        new[0] = 0
        self.assertEqual(computeResidual(old,new,new_thetas),0.25)

        new = range(360)
        new = new[90:]+new[:90]
        self.assertEqual(computeResidual(old,new,new),0)

    def matrices_equal(self, array1, array2):
        return np.allclose(array1, array2, atol=1e-03)

    def test_cartesian(self):
        self.assertTrue(np.allclose(cartesian(1,0),(1,0)))
        self.assertTrue(np.allclose(cartesian(1,90),(0,1)))
        self.assertTrue(np.allclose(cartesian(1,180),(-1,0)))
        self.assertTrue(np.allclose(cartesian(1,270),(0,-1)))
        self.assertTrue(np.allclose(cartesian(1,360),(1,0)))

    def test_rotate_cart(self):
        self.assertTrue(np.allclose(rotate_cart(1,0,0,1),(0,1)))
        self.assertTrue(np.allclose(rotate_cart(1,0,-1,0),(-1,0)))
        self.assertTrue(np.allclose(rotate_cart(1,0,0,-1),(0,-1)))

    def test_translate(self):
        rs_in = range(1,361)
        thetas_in = range(360)
        thetas_in[0] = 360
        dx, dy, dtheta = 0,0,90
        expected_thetas = thetas_in[90:]+thetas_in[:90]
        rs_out, thetas_out = translate(rs_in, dx, dy, dtheta)
        self.assertTrue(self.matrices_equal(rs_out,rs_in))
        self.assertTrue(self.matrices_equal(thetas_out,expected_thetas))

        rs_in = [0]*360
        rs_in[0] = 1
        dx,dy,dtheta = 1,1,0
        rs_expected = [np.sqrt(2)]*360
        rs_expected[0] = np.sqrt(5)
        expected_thetas = [45]*360
        expected_thetas[0] = np.degrees(np.arctan2(1,2))
        rs_out, thetas_out = translate(rs_in, dx, dy, dtheta)
        self.assertTrue(self.matrices_equal(rs_out,rs_expected))
        self.assertTrue(self.matrices_equal(thetas_out,expected_thetas))

        dx,dy,dtheta = 1,1,90
        expected_thetas = [e+90 for e in expected_thetas]
        rs_out, thetas_out = translate(rs_in, dx, dy, dtheta)
        self.assertTrue(self.matrices_equal(rs_out,rs_expected))
        self.assertTrue(self.matrices_equal(thetas_out,expected_thetas))

    def test_residual(self):
        old = range(1,361)
        new = range(1,361)
        self.assertAlmostEqual(residual(old,new,0,0,0),0)

        old = range(1,361)
        new2 = range(1,361)
        new2 = new2[90:]+new2[:90]
        self.assertAlmostEqual(residual(old,new2,0,0,90),0)

        old = range(1,361)
        new2[0] = 1
        self.assertAlmostEqual(residual(old,new2,0,0,90),90**2)


if __name__ == '__main__':
    unittest.main()
