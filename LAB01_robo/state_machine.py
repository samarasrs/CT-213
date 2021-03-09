import random
import math
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.cont_time = 0.0


    def check_transition(self, agent, state_machine):
        if (self.cont_time*SAMPLE_TIME) > MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())


    def execute(self, agent):
        agent.set_velocity(FORWARD_SPEED, 0.0)
        self.cont_time += 1


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.cont_time = 0.0
        self.radius = INITIAL_RADIUS_SPIRAL
        self.angularSpeed = 0.0
    
    def check_transition(self, agent, state_machine):
        if (self.cont_time * SAMPLE_TIME) > MOVE_IN_SPIRAL_TIME:
            state_machine.change_state(MoveForwardState())
        if agent.get_bumper_state():
            state_machine.change_state(GoBackState())

    def execute(self, agent):
        self.radius = INITIAL_RADIUS_SPIRAL + SPIRAL_FACTOR * self.cont_time * SAMPLE_TIME
        self.angularSpeed = math.sqrt((FORWARD_SPEED ** 2 - SPIRAL_FACTOR ** 2)/self.radius ** 2)
        agent.set_velocity(FORWARD_SPEED, self.angularSpeed)
        self.cont_time += 1



class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.cont_time = 0.0

    def check_transition(self, agent, state_machine):
        if (self.cont_time * SAMPLE_TIME) > GO_BACK_TIME:
            state_machine.change_state(RotateState())

    def execute(self, agent):
        agent.set_velocity(BACKWARD_SPEED, 0.0)
        self.cont_time += 1


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.cont_time = 0.0
        self.angle = random.uniform(-math.pi, math.pi)
        self.rotateTime = abs(self.angle)/ANGULAR_SPEED



    def check_transition(self, agent, state_machine):
        if (self.cont_time * SAMPLE_TIME) > self.rotateTime:
            state_machine.change_state(MoveForwardState())

    
    def execute(self, agent):
        if self.angle >= 0:
            agent.set_velocity(0.0, ANGULAR_SPEED)
        else:
            agent.set_velocity(0.0, -ANGULAR_SPEED)
        self.cont_time += 1