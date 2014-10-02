import numpy


def jacobian(func, vector):
    n = len(vector)
    f_x = func(vector)
    epsilon = 1e-8

    J = numpy.zeros((n, n))

    for i in range(n):
        perturbation = numpy.zeros((n))
        perturbation[i] = epsilon
        J[:, i] = (func(vector + perturbation) - f_x) / epsilon

    return J
