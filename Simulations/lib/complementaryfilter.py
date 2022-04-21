"""

Author: Francisco Branco
Created: 08/04/2022
Description: Complementary Filter for better compensation of AUV

"""


import numpy as np
import utils


class ComplementaryFilter:
    def __init__(self, H_matrix=None, state_history=False, dt=1):
        self.dt = dt
        self.state_history = state_history

        self.A_matrix = np.array([[0, 0, 1, 0],
                                  [0, 0, 0, 1],
                                  [0, 0, 0, 0],
                                  [0, 0, 0, 0]])

        self.B_matrix = np.array([[1, 0],
                                  [0, 1],
                                  [0, 0],
                                  [0, 0]])
        
        self.C_matrix = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0]])

        # Gain matrix
        if H_matrix is None:
            self.H_matrix = np.array([[1, 0],
                                      [0, 1],
                                      [1, 0],
                                      [0, 1]])
        else:
            self.H_matrix = H_matrix

        self.state = {"x_pred": 0,
                      "y_pred": 0,
                      "vcx_pred": 0,
                      "vcy_pred": 0}

        self.inputs = {"x_EKF": 0,
                       "y_EKF": 0,
                       "vx_dop": 0,
                       "vy_dop": 0}

        if state_history:
            self.past_state = self.state.copy()
            for key in self.past_state.keys():
                self.past_state[key] = []

    def outputs(self):
        x = np.array([self.state["x_pred"],
                      self.state["y_pred"],
                      self.state["vcx_pred"],
                      self.state["vcy_pred"]])

        outputs = np.matmul(self.C_matrix, x)
        #print(x)
        #print(outputs)

        return outputs

    def inputs_outputs(self):
        outputs = {"x_pred": self.state["x_pred"], "y_pred": self.state["y_pred"]}

        return self.inputs.copy(), outputs

    def set_initial_conditions(self, ic):
        for key in ic.keys():
            self.state[key] = ic[key]

        self.inputs["x_EKF"] = self.state["x_pred"]
        self.inputs["y_EKF"] = self.state["y_pred"]

    def cf_update(self, inputs=None, dt=None):
        if dt == None:
            dt = self.dt
        if inputs is not None:
            for key in self.inputs.keys():
                if inputs[key] != None:
                    self.inputs[key] = inputs[key]
        
        x = np.array([self.state["x_pred"],
                      self.state["y_pred"],
                      self.state["vcx_pred"],
                      self.state["vcy_pred"]])

        u = np.array([self.inputs["vx_dop"], self.inputs["vy_dop"]])

        measure = np.array([self.inputs["x_EKF"],
                            self.inputs["y_EKF"]])

        outputs = self.outputs()

        # print(self.inputs["vx_dop"])

        x_dot = np.matmul(self.A_matrix, x) + np.matmul(self.B_matrix, u) + np.matmul(self.H_matrix, measure - outputs)
        # print("measure ", measure)
        # print("outputs ", outputs)
        # print("difference ", measure - outputs)
        # print("H_matrix result ", np.matmul(self.H_matrix, measure - outputs))
        # print("x ", x)
        # print("x_dot ", x_dot)

        i = 0
        for key in self.state.keys():
            self.state[key] = self.state[key] + x_dot[i] * self.dt
            i = i + 1

        if self.state_history:
            for key in self.state.keys():
                self.past_state[key].append(self.state[key])


class DopplerMeasureSimulation:
    def __init__(self, variance, sampling_period=1):
        self.variance = variance
        self.sampling_period = sampling_period
        self.last_time = 0

    def measurement(self, current_time, angle=0, velocity=1):
        if self.last_time == 0 or current_time - self.last_time >= self.sampling_period:
            self.last_time = current_time
            
            if isinstance(self.variance, np.ndarray):
                v_w = velocity + np.random.normal(0, self.variance)
            else:
                v_noise = velocity + np.random.normal(0, self.variance)
                v_w = np.array([v_noise * np.cos(angle), v_noise * np.sin(angle)])
        else:
            v_w = [None, None]
        return v_w