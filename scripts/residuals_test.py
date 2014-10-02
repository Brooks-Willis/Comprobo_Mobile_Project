import unittest
import numpy
from residuals import makeFunc, wrapInterp, interpaLazers, computeResidual, translate


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

    def test_translate(self):
        new = range(360)
        dx, dy, dtheta = 0,0,90
        expected = new[270:]+new[:270]
        self.assertEqual(translate(new, dx, dy, dtheta),(expected,new))

if __name__ == '__main__':
    unittest.main()
