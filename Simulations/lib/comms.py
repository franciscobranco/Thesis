"""

Author: Francisco Branco
Created: 19/05/2021

"""


import numpy as np
from math import pi
import utils


class CommunicationBlock:
    """
    Communication type determines what kind of output is calculated:
        - 0 for continuous communication
    """
    def __init__(self, params, id, num_auv, communication_matrix=None, speed_profile=1, delay=0, state_history=False, dt=1):
        self.id = id
        self.speed_profile = speed_profile
        self.delay = delay
        self.state_history = state_history
        self.dt = dt

        self.num_auv = num_auv
        self.communication_matrix = communication_matrix
        self.D_matrix = np.zeros(communication_matrix.shape)
        
        for i in range(self.num_auv):
            self.D_matrix[i][i] = sum(communication_matrix[0])

        self.L = self.D_matrix - self.communication_matrix
        self.Ls = (1/2) * (self.L + self.L.T)

        eigenvalues, _ = np.linalg.eig(self.Ls)
        if len(eigenvalues) > 1:
            cut_eigenvalues = np.delete(eigenvalues, np.argmin(eigenvalues))

        self.params = {}
        
        self.params["lambda2"] = cut_eigenvalues.min()
        self.params["lambdaN"] = cut_eigenvalues.max()

        # h function parameters
        self.params["c0"] = params["c0"]
        self.params["c1"] = params["c1"]
        self.params["c2"] = params["c2"]

        # state equation parameters
        self.params["l"] = params["l"]
        self.params["k"] = params["k"]
        self.params["epsilon"] = params["epsilon"]
        self.params["epsilon0"] = params["epsilon0"]
        self.params["theta"] = params["theta"]
        self.params["alpha"] = self.params["l"] / self.params["lambda2"]
        self.params["sigma"] = (1 - self.params["epsilon"]) * self.params["k"] - self.params["alpha"]
        self.params["sigma_i"] = (self.params["k"] * self.D_matrix[self.id][self.id] / self.params["epsilon"]) + 2 * self.params["alpha"] * self.params["lambdaN"]

        self.inputs = {}
        self.state = {}

        if self.state_history:
            self.past_state = {}
            for i in range(self.num_auv):
                self.past_state["gamma" + str(i)] = []
            self.past_state["broadcasts"] = []

    def comm_update(self, dt=None):
        if dt is None:
            dt = self.dt

        for i in range(self.num_auv):
            self.state["gamma" + str(i)] = self.state["gamma" + str(i)] + self.speed_profile * dt

        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])
    
    def cpf_output(self):
        return self.state.copy()

    def inputs_outputs(self):
        outputs = self.cpf_output()
        
        return (self.inputs.copy(), outputs)

    def set_initial_conditions(self, gammas):
        for key in gammas.keys():
            self.state[key] = gammas[key]

    def check_update(self, t, error, actual_state):
        # or self.state_trigger(error) > 0
        if self.time_trigger(t, error) > 0:
            if self.state_history:
                self.past_state["broadcasts"].append(t)
            self.state["gamma" + str(self.id)] = actual_state
            state = {}
            for i in range(self.num_auv):
                state["gamma" + str(i)] = self.state["gamma" + str(self.id)]
            return state
        else:
            return -1

    def reset(self, broadcast):
        for i in range(self.num_auv):
            if i != self.id:
                self.state["gamma" + str(i)] = broadcast["gamma" + str(i)]

    def time_trigger(self, t, error):
        h = self.params["c0"] + self.params["c1"] * np.exp(- self.params["c2"] * t)
        delta = np.abs(error) - h

        return delta

    def state_trigger(self, error):
        error_sum = 0
        for j in range(self.num_auv):
            error_sum = error_sum + self.communication_matrix[self.id][j] * np.power(self.state["gamma" + str(self.id)] - self.state["gamma" + str(j)], 2)
        delta = np.power(error, 2) - (self.params["theta"] * (self.params["sigma"] / self.params["sigma_i"]) * error_sum + self.params["epsilon0"])
        
        return delta

    def simple_delay_output(self, t):
        if t - self.start_time < self.delay:
            
            self.start_time = t
        return