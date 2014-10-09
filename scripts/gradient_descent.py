import numpy
from gradient import gradient


def gradient_descent(func, v):
    epsilon = 0.001
    absolute_error = 1e-10

    v_old = numpy.array(v)
    v_old = make_column_vector(v_old)
    v_new = v_old

    f_old = func(v_old)
    f_new = f_old + absolute_error * 2

    while numpy.absolute(f_new - f_old) > absolute_error:
        v_old = v_new
        f_old = f_new

        G = gradient(func, v_old)
        v_new = v_old - epsilon * G
        f_new = func(v_new)

        v_new, f_new, epsilon = adapt_learning_rate(
            v_old, v_new, f_old, f_new, epsilon
        )

        print("hello " + str(epsilon))

    return v_new


def make_column_vector(vector):
    if not_one_dimensional(vector):
        raise Exception

    return vector.reshape(max(vector.shape), 1)


def not_one_dimensional(vector):
    return (1 not in vector.shape) or (len(vector.shape) != 2)


def adapt_learning_rate(v_old, v_new, f_old, f_new, epsilon):
    print "adapt_learning_rate: " + str(v_old)
    if f_new > f_old:
        return v_old, f_old, epsilon * 0.2
    else:
        return v_new, f_new, epsilon * 1.2
