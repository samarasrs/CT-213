import numpy as np


def gradient_descent(cost_function, gradient_function, theta0, alpha, epsilon, max_iterations):
    """
    Executes the Gradient Descent (GD) algorithm to minimize (optimize) a cost function.

    :param cost_function: function to be minimized.
    :type cost_function: function.
    :param gradient_function: gradient of the cost function.
    :type gradient_function: function.
    :param theta0: initial guess.
    :type theta0: numpy.array.
    :param alpha: learning rate.
    :type alpha: float.
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
        grad = gradient_function(theta)
        theta_aux = np.array([0.0, 0.0])
        theta_aux[0] = theta[0] - alpha * grad[0]
        theta_aux[1] = theta[1] - alpha * grad[1]
        history.append(theta_aux)
        theta = theta_aux
        cost = cost_function(theta)
        n += 1

    return theta, history

