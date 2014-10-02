import unittest
import numpy
from residuals import makeFunc, wrapInterp, interpaLazers, computeResidual, translate, cartesian, rotate_cart


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
        self.assertEqual(computeResidual(old,new,new_thetas),0.5)

    def matrices_equal(self, array1, array2):
        return numpy.allclose(array1, array2, atol=1e-03)

    def test_cartesian(self):
        self.assertTrue(cartesian(1,0),(0,1))
        self.assertTrue(cartesian(1,90),(1,0))
        self.assertTrue(cartesian(1,180),(0,-1))
        self.assertTrue(cartesian(1,270),(-1,0))
        self.assertTrue(cartesian(1,360),(0,1))

    def test_rotate_cart(self):
        self.assertTrue(rotate_cart(0,1,1,0),(1,0))

    def test_translate(self):
        #breaks on 0
        rs_in = range(1,361)
        thetas_in = range(360)
        dx, dy, dtheta = 0,0,90
        expected_thetas = thetas_in[270:]+thetas_in[:270]
        rs_out, thetas_out = translate(rs_in, dx, dy, dtheta)
        print thetas_out
        # print expected_thetas
        self.assertTrue(self.matrices_equal(rs_out,rs_in))
        self.assertTrue(self.matrices_equal(thetas_out,expected_thetas))


if __name__ == '__main__':
    unittest.main()
