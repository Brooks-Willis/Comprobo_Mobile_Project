import unittest
import numpy
from gradient_descent import gradient_descent


class TestGradientDescent(unittest.TestCase):
    def parabola(self, v):
        x = v[0][0]
        return (x - 1) ** 2

    def elliptic_paraboloid(self, v):
        x = v[0][0]
        y = v[1][0]
        return (x + 1) ** 2 + (y + 2) ** 2

    def test_gradient_descent(self):
        initial_guess = numpy.array([[3]])
        v = gradient_descent(self.parabola, initial_guess)
        x = v[0, 0]
        self.assertAlmostEqual(x, 1, delta=0.01)

        initial_guess = numpy.array([[4], [2]])
        v = gradient_descent(self.elliptic_paraboloid, initial_guess)
        x = v[0, 0]
        y = v[1, 0]
        self.assertAlmostEqual(x, -1, delta=0.01)
        self.assertAlmostEqual(y, -2, delta=0.01)

if __name__ == '__main__':
    unittest.main()
