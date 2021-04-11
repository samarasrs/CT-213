import numpy as np
import random
from math import inf


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement
        delta = upper_bound - lower_bound
        self.x = np.random.uniform(lower_bound, upper_bound)
        self.v = np.random.uniform(-delta, delta)
        self.J = None
        self.best_interation = self.x
        self.Jbest_interation = -inf


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.w = hyperparams.inertia_weight
        self.phip = hyperparams.cognitive_parameter
        self.phig = hyperparams.social_parameter
        self.num_particles = hyperparams.num_particles
        self.particles = []
        self.vlimite = self.upper_bound - self.lower_bound
        for i in range(self.num_particles):
            self.particles.append(Particle(self.lower_bound, self.upper_bound))

        self.best = None
        self.Jbest = - inf

        self.cont = 0
        self.i = 0

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.best

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        # Todo: implement
        return self.Jbest

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        position = self.particles[self.cont].x

        return position

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Todo: implement
        print(self.i, "cont: " ,self.cont,"x: ", self.particles[self.cont].x)
        rp = random.uniform(0.0, 1.0)
        rg = random.uniform(0.0, 1.0)
        new_particle = Particle(self.lower_bound, self.upper_bound)

        if self.particles[self.cont].J > self.particles[self.cont].Jbest_interation:
            new_particle.best_interation = self.particles[self.cont].x
            new_particle.Jbest_interation = self.particles[self.cont].J
        else:
            new_particle.best_interation = self.particles[self.cont].best_interation
            new_particle.Jbest_interation = self.particles[self.cont].Jbest_interation

        if self.particles[self.cont].J > self.Jbest:
            self.Jbest = self.particles[self.cont].J
            self.best = self.particles[self.cont].x

        new_particle.v = (self.w * self.particles[self.cont].v +
                          self.phip * rp * (self.particles[self.cont].best_interation - self.particles[self.cont].x) +
                          self.phig * rg * (self.best - self.particles[self.cont].x))

        for i in range(np.size(new_particle.v)):
            new_particle.v[i] = min(max(new_particle.v[i], - self.vlimite[i]), self.vlimite[i])

        new_particle.x = self.particles[self.cont].x + new_particle.v

        for i in range(np.size(new_particle.x)):
            new_particle.x[i] = min(max(new_particle.x[i], self.lower_bound[i]), self.upper_bound[i])

        self.particles[self.cont] = new_particle
        self.cont = (self.cont + 1) % self.num_particles
        self.i = self.i + 1

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement

        self.particles[self.cont].J = value
        self.advance_generation()

