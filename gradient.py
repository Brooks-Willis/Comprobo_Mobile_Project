import numpy


def gradient(func, vector):
	m, _ = numpy.shape(vector)
	f_v = func(vector)
	epsilon = 1e-10

	G = numpy.zeros((m, 1))

	for i in range(m):
		perturbation = numpy.zeros((m, 1))
		perturbation[i, 0] = epsilon
		G[i, 0] = (func(vector + perturbation) - f_v) / epsilon

	return G
