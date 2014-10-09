import unittest
import numpy
from gradient import gradient


class TestGradient(unittest.TestCase):
    def parabola(self, vector):
        x = vector[0, 0]
        return (x - 1) ** 2

    def elliptic_paraboloid(self, v):
        x = v[0][0]
        y = v[1][0]
        return (x + 1) ** 2 + (y + 2) ** 2

    def matrices_equal(self, array1, array2):
        return numpy.allclose(array1, array2, rtol=1e-05, atol=1e-08)

    def test_gradient(self):
        initial_guess = numpy.array([[3]])
        derivative = gradient(self.parabola, initial_guess)
        self.assertTrue(self.matrices_equal(derivative, [[4]]))

        initial_guess = numpy.array([[1]])
        derivative = gradient(self.parabola, initial_guess)
        self.assertTrue(self.matrices_equal(derivative, [[0]]))

        initial_guess = numpy.array([[1], [2]])
        derivative = gradient(self.elliptic_paraboloid, initial_guess)
        self.assertTrue(self.matrices_equal(derivative, [[4], [8]]))

if __name__ == '__main__':
    unittest.main()
