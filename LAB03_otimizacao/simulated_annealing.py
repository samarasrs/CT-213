from math import exp
import random
import numpy as np


def simulated_annealing(cost_function, random_neighbor, schedule, theta0, epsilon, max_iterations):
    """
    Executes the Simulated Annealing (SA) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param random_neighbor: function which returns a random neighbor of a given point.
    :type random_neighbor: numpy.array.
    :param schedule: function which computes the temperature schedule.
    :type schedule: function.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param epsilon: used to stop the optimization if the current cost is less than epsilon.
    :type epsilon: float.
    :param max_iterations: maximum number of iterations.
    :type max_iterations: int.
    :return theta: local minimum.
    :rtype theta: np.array.
    :return history: history of points visited by the algorithm.
    :rtype history: list of np.array.
    """
    theta = theta0
    history = [theta0]
    cost = cost_function(theta)
    n = 0

    while cost >= epsilon and n < max_iterations:
        theta_aux = np.array([0.0, 0.0])
        theta_aux[0] = theta[0]
        theta_aux[1] = theta[1]
        temperature = schedule(n)
        if temperature < 0.0:
            history.append(theta_aux)
            break
        neighbor = random_neighbor(theta_aux)
        delta_e = cost_function(theta_aux) - cost_function(neighbor) #estamos minimizando
        if delta_e > 0:
            theta_aux = neighbor
        else:
            r = random.uniform(0.0, 1.0)
            if r <= exp(delta_e / temperature):
                theta_aux = neighbor
        history.append(theta_aux)
        n += 1
        theta = theta_aux
        cost = cost_function(theta)

    return theta, history
