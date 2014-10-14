import unittest
import numpy
from gradient_descent import (gradient_descent, 
                              make_column_vector,
                              adapt_learning_rate
                             )


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
        v = gradient_descent(self.parabola, initial_guess, abs_error = 1e-10)
        x = v[0, 0]
        self.assertAlmostEqual(x, 1, delta=0.01)

        # initial_guess = numpy.array([[4], [2]])
        # v = gradient_descent(self.elliptic_paraboloid, initial_guess)
        # x = v[0, 0]
        # y = v[1, 0]
        # self.assertAlmostEqual(x, -1, delta=0.01)
        # self.assertAlmostEqual(y, -2, delta=0.01)

    def test_column_vector(self):
        expected = numpy.array([[1], [2]])

        x = numpy.array([[1], [2]])
        self.assertTrue(numpy.array_equal(expected, make_column_vector(x)))

        x = numpy.array([[1, 2]])
        self.assertTrue(numpy.array_equal(expected, make_column_vector(x)))

        x = numpy.array([[1, 2], [3, 4]])
        self.assertRaises(Exception, make_column_vector, x)

        x = numpy.array([1, 2])
        self.assertRaises(Exception, make_column_vector,  x)

    def test_adapt_learning_rate(self):
        v_old = 0
        v_new = 1
        f_old = 2
        f_new = 1
        epsilon = 1
        new_v, new_f, new_learning_rate = adapt_learning_rate(
            v_old, v_new, f_old, f_new, epsilon
        )

        self.assertEqual(new_v, v_new)
        self.assertEqual(new_f, f_new)
        self.assertEqual(new_learning_rate, 1.2)

        f_old = 0
        new_v, new_f, new_learning_rate = adapt_learning_rate(
            v_old, v_new, f_old, f_new, epsilon
        )

        self.assertEqual(new_v, v_old)
        self.assertEqual(new_f, f_old)
        self.assertEqual(new_learning_rate, 0.2)

if __name__ == '__main__':
    unittest.main()
