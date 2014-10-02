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

    return v_new

def make_column_vector(v):
    return v.reshape(v.shape[0], -1)