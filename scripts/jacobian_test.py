import unittest
import numpy
from jacobian import jacobian


class TestJacobian(unittest.TestCase):
    def setUp(self):
        self.true_jacobian = numpy.array([[1, 1], [1, -1]])

    def random_function(self, (x, y)):        
        f_x = x + y
        f_y = x - y
        return numpy.array([f_x, f_y])

    def matrices_equal(self, array1, array2):
        return numpy.allclose(array1, array2, rtol=1e-05, atol=1e-08)

    def test_jacobian(self):
        J = jacobian(self.random_function, [1, 2])
        self.assertTrue(self.matrices_equal(J, self.true_jacobian))

if __name__ == '__main__':
    unittest.main()
