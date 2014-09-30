import unittest
import numpy
from residuals import makeFunc, wrapInterp, interpolateLazers


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

    def test_interpolateLazers(self):
        interp = interpolateLazers(range(0, 360))
        self.assertEqual(interp(359.5), 179.5)
        self.assertEqual(interp(182.2365),182.2365)

if __name__ == '__main__':
    unittest.main()
