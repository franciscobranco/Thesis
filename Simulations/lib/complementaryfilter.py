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

        if inputs["x_EKF"] == None:
            x_dot = np.matmul(self.A_matrix, x) + np.matmul(self.B_matrix, u)
        else:
            x_dot = np.matmul(self.A_matrix, x) + np.matmul(self.B_matrix, u) + np.matmul(self.H_matrix, measure - outputs)

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







class ComplementaryKalmanFilter:
    def __init__(self, A_matrix=None, B_matrix=None, C_matrix=None, Q_matrix=None, Dopvar=None, state_history=False, dt=1):
        self.dt = dt
        if A_matrix is None:
            self.A_matrix = np.array([[1, 0, dt, 0],
                                      [0, 1, 0, dt],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]])
        else:
            self.A_matrix = A_matrix
        if B_matrix is None:
            self.B_matrix = np.array([[dt, 0],
                                      [0, dt],
                                      [0, 0],
                                      [0, 0]])
        else:
            self.B_matrix = B_matrix
        if C_matrix is None:
            self.C_matrix = np.array([[1, 0, 0, 0],
                                      [0, 1, 0, 0]])
        else:
            self.C_matrix = C_matrix
        if Q_matrix is None:
            self.Q_matrix = 10 * np.exp(-6) * np.array([[10, 0, 0, 0],
                                                        [0, 10, 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]])
        else:
            self.Q_matrix = Q_matrix
        if Dopvar is None:
            self.Dopvar_matrix = np.array([[0.1, 0, 0, 0],
                                           [0, 0.1, 0, 0],
                                           [0, 0, 0, 0],
                                           [0, 0, 0, 0]])
        else:
            self.Dopvar_matrix = np.array([[Dopvar, 0, 0, 0],
                                           [0, Dopvar, 0, 0],
                                           [0, 0, 0, 0],
                                           [0, 0, 0, 0]])

        self.P_aposteriori = np.zeros((4, 4))

        self.state = {
            "x": 0,
            "y": 0,
            "vc_x": 0,
            "vc_y": 0
        }
        
        self.inputs = {
            "vx_dop": 0,
            "vy_dop": 0,
            "x_EKF": 0,
            "y_EKF": 0,
            "R": None
        }

        if state_history:
            self.past_state = self.state.copy()
            for key in self.past_state.keys():
                self.past_state[key] = []

    def set_initial_conditions(self, ic):
        for key in self.state.keys():
            self.state[key] = ic[key]

    def inputs_outputs(self):
        outputs = {"x": self.state["x"], "y": self.state["y"]}
        return self.inputs.copy(), outputs

    def ckf_update(self, inputs, t):
        for input in self.inputs.keys():
            self.inputs[input] = inputs[input]

        self.prediction()
        self.update()

        for state in self.state.keys():
            self.past_state[state].append(self.state[state])

    def prediction(self):
        vel_m = np.array([self.inputs["vx_dop"], self.inputs["vy_dop"]])
        state = np.array([self.state["x"], self.state["y"], self.state["vc_x"], self.state["vc_y"]])
        state = np.matmul(self.A_matrix, state.T) + np.matmul(self.B_matrix, vel_m)
        
        self.state["x"] = state[0]
        self.state["y"] = state[1]
        self.state["vc_x"] = state[2]
        self.state["vc_y"] = state[3]

        if len(self.past_state["x"]) != 0:
            self.P_apriori = np.matmul(self.A_matrix, np.matmul(self.P_aposteriori, self.A_matrix.T)) + self.Q_matrix + self.Dopvar_matrix
        else:
            self.P_apriori = self.Q_matrix + self.Dopvar_matrix

    def update(self):
        state = np.array([self.state["x"], self.state["y"], self.state["vc_x"], self.state["vc_y"]])
        
        ekf = np.zeros((2,))
        #print(self.inputs["R"])
        if self.inputs["R"] is not None:
            ekf[0] = self.inputs["x_EKF"]
            ekf[1] = self.inputs["y_EKF"]
            R = self.inputs["R"]
            #print(np.matmul(self.C_matrix, np.matmul(self.P_apriori, self.C_matrix.T)))
            K_k = np.matmul(self.P_apriori, (np.matmul(self.C_matrix.T, np.linalg.inv(np.matmul(self.C_matrix, np.matmul(self.P_apriori, self.C_matrix.T)) + R))))
        else:
            ekf[0] = 0
            ekf[1] = 0
            R = np.zeros((2, 2))
            K_k = np.zeros((4, 2))
        
        state = state + np.matmul(K_k, (ekf - np.matmul(self.C_matrix, state)))
        self.P_aposteriori = np.matmul(np.identity(4) - np.matmul(K_k, self.C_matrix), self.P_apriori)
        
        self.state["x"] = state[0]
        self.state["y"] = state[1]
        self.state["vc_x"] = state[2]
        self.state["vc_y"] = state[3]