from math import inf
import numpy as np


def hill_climbing(cost_function, neighbors, theta0, epsilon, max_iterations):
    """
    Executes the Hill Climbing (HC) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param neighbors: function which returns the neighbors of a given point.
    :type neighbors: list of numpy.array.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: numpy.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of numpy.array.
    """
    theta = theta0
    history = [theta0]
    cost = cost_function(theta)
    n = 0
    while cost >= epsilon and n < max_iterations:
        neighbors_list = neighbors(theta)
        theta_aux = np.array([0.0, 0.0])
        theta_aux[0] = theta[0]
        theta_aux[1] = theta[1]
        best = np.array([0.0, 0.0])
        best[0] = neighbors_list[0][0]
        best[1] = neighbors_list[0][1]
        for neighbor in neighbors_list:
            if cost_function(neighbor) < cost_function(best):
                best[0] = neighbor[0]
                best[1] = neighbor[1]
        if cost_function(best) > cost_function(theta_aux):
            history.append(theta_aux)
        else:
            history.append(best)
            theta = best
        n += 1

    return theta, history
