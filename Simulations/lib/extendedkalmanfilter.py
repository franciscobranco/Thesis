"""

Author: Francisco Branco
Created: 19/01/2022
Description: Extended Kalman Filter module for target pursuit

"""


import utils
from math import pi
import numpy as np


class ExtendedKalmanFilter:
    def __init__(self, F_matrix=None, Q_matrix=None, R_matrix=None, dt=1):
        self.dt = dt

        if F_matrix is None:
            self.F_matrix = np.array([[1, 0, self.dt, 0],
                                      [0, 1, 0, self.dt],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]])
        else:
            self.F_matrix = F_matrix
        if Q_matrix is None:
            self.Q_matrix = 10 * np.exp(-6) * np.array([[10, 0, 0, 0],
                                                        [0, 10, 0, 0],
                                                        [0, 0, 1, 0],
                                                        [0, 0, 0, 1]])
        else:
            self.Q_matrix = Q_matrix
        if R_matrix is None:
            self.R_matrix = 1
        else:
            self.R_matrix = R_matrix

        self.P_aposteriori = np.zeros((4, 4))

        self.state = {
            "x": 0,
            "y": 0,
            "x_dot": 0,
            "y_dot": 0
        }
        
        if isinstance(R_matrix, np.ndarray):
            self.inputs = {}
            for i in range(R_matrix.shape[0]):
                self.inputs["tracker_x" + str(i)] = 0
                self.inputs["tracker_y" + str(i)] = 0
                self.inputs["range" + str(i)] = 0
        else:
            self.inputs = {
                "tracker_x0": 0,
                "tracker_y0": 0,
                "range0": 0
            }

        self.past_state = self.state.copy()
        for key in self.past_state.keys():
            self.past_state[key] = []

    def set_initial_conditions(self, ic):
        for key in self.state.keys():
            self.state[key] = ic[key]
        #self.last_state = self.state.copy()

    def inputs_outputs(self):
        outputs = self.state.copy()
        outputs["theta"], outputs["theta_dot"] = self.get_thetas()
        return self.inputs.copy(), outputs

    def ekf_update(self, inputs, t):
        for input in self.inputs.keys():
            self.inputs[input] = inputs[input]

        self.prediction()
        self.update()
        #if np.abs(self.state["x_dot"]) > 1 or np.abs(self.state["y_dot"]) > 1:
        #    print(t)
        #print(self.state)
        #print("apriori")
        #print(self.P_apriori)
        #print("aposteriori")
        #print(self.P_aposteriori)

        for state in self.state.keys():
            self.past_state[state].append(self.state[state])

    def prediction(self):
        state = np.array([self.state["x"], self.state["y"], self.state["x_dot"], self.state["y_dot"]])
        state = np.matmul(self.F_matrix, state.T)
        
        
        self.state["x"] = state[0]
        self.state["y"] = state[1]
        self.state["x_dot"] = state[2]
        self.state["y_dot"] = state[3]

        if len(self.past_state["x"]) != 0:
            self.P_apriori = np.matmul(self.F_matrix, np.matmul(self.P_aposteriori, self.F_matrix.T)) + self.Q_matrix
            #print(self.P_aposteriori)
            #print(np.matmul(self.P_aposteriori, self.F_matrix.T))
        else:
            self.P_apriori = self.Q_matrix

    def update(self):
        d_k = self.distance()
        #C_k = np.array([(self.state["x"] - self.inputs["tracker_x"]) / d_k, (self.state["y"] - self.inputs["tracker_y"]) / d_k, 0, 0])
        state = np.array([self.state["x"], self.state["y"], self.state["x_dot"], self.state["y_dot"]])
        #print(C_k)
        #print(self.P_apriori)
        #print(self.P_apriori.dot(C_k.T))
        #print(C_k.dot(self.P_apriori.dot(C_k.T)))
        #print(np.power(C_k.dot(self.P_apriori.dot(C_k.T)) + self.R_matrix, -1))
        
        if isinstance(self.R_matrix, np.ndarray):
            C_k = np.zeros((self.R_matrix.shape[0], 4))
            ranges = np.zeros((self.R_matrix.shape[0],))
            for i in range(self.R_matrix.shape[0]):
                if d_k[i] == 0:
                    C_k[i][0] = 0
                    C_k[i][1] = 0
                else:
                    C_k[i][0] = (self.state["x"] - self.inputs["tracker_x" + str(i)]) / d_k[i]
                    C_k[i][1] = (self.state["y"] - self.inputs["tracker_y" + str(i)]) / d_k[i]
                C_k[i][2] = 0
                C_k[i][3] = 0
                if self.inputs["range" + str(i)] != None:
                    ranges[i] = self.inputs["range" + str(i)]
                else:
                    ranges[i] = -1
            
            K_k = np.matmul(self.P_apriori, (np.matmul(C_k.T, np.linalg.inv(np.matmul(C_k, np.matmul(self.P_apriori, C_k.T)) + self.R_matrix))))
            for i in range(self.R_matrix.shape[0]):
                if ranges[i] < 0:
                    K_k[0][i] = 0
                    K_k[1][i] = 0
                    K_k[2][i] = 0
                    K_k[3][i] = 0
                    
            
            state = state + np.matmul(K_k, (ranges - d_k))
            self.P_aposteriori = np.matmul(np.identity(4) - np.matmul(K_k, C_k), self.P_apriori)
        
        else:
            if d_k == 0:
                C_k = np.array([0, 0, 0, 0])
            else:
                C_k = np.array([(self.state["x"] - self.inputs["tracker_x0"]) / d_k, (self.state["y"] - self.inputs["tracker_y0"]) / d_k, 0, 0])
            K_k = np.matmul(self.P_apriori, (C_k.T * np.power(np.matmul(C_k, np.matmul(self.P_apriori, C_k.T)) + self.R_matrix, -1)))# self.P_apriori.dot(C_k.T.dot(np.power(C_k.dot(self.P_apriori.dot(C_k.T)) + self.R_matrix, -1)))

            
            if self.inputs["range0"] == None:
                K_k[0] = 0
                K_k[1] = 0
                K_k[2] = 0
                K_k[3] = 0

                ranges = -1
            else:
                ranges = self.inputs["range0"]

            state = state + K_k * (ranges - d_k)
            self.P_aposteriori = np.matmul(np.identity(4) - np.outer(K_k, C_k), self.P_apriori)
        
        self.state["x"] = state[0]
        self.state["y"] = state[1]
        self.state["x_dot"] = state[2]
        self.state["y_dot"] = state[3]
        
    def distance(self):
        q_k = np.array([self.state["x"], self.state["y"]])
        if isinstance(self.R_matrix, np.ndarray):
            d_k = np.zeros((self.R_matrix.shape[0],))
            for i in range(self.R_matrix.shape[0]):
                p_k = np.array([self.inputs["tracker_x" + str(i)], self.inputs["tracker_y" + str(i)]])
                d_k[i] = np.linalg.norm(p_k - q_k)
        else:
            p_k = np.array([self.inputs["tracker_x0"], self.inputs["tracker_y0"]])
            d_k = np.linalg.norm(p_k - q_k)
        
        return d_k

    def get_thetas(self):
        if len(self.past_state["x"]) >= 2:
            theta_1 = np.arctan2(self.past_state["x"][-1] - self.past_state["x"][-2], self.past_state["y"][-1] - self.past_state["y"][-2])
            if len(self.past_state["x"]) >= 3:
                theta_2 = np.arctan2(self.past_state["x"][-2] - self.past_state["x"][-3], self.past_state["y"][-2] - self.past_state["y"][-3])
                omega = (theta_1 - theta_2) / self.dt
                return theta_1, omega
            return theta_1, 0
        return 0, 0




class RangeMeasureSimulation:
    def __init__(self, R=1, sampling_period=1):
        self.R = R
        self.sampling_period = sampling_period
        self.last_time = 0

    def measurement(self, current_time, tracker, target):
        if self.last_time == 0 or current_time - self.last_time >= self.sampling_period:
            self.last_time = current_time
            y_k = self.distance(tracker, target) + np.random.normal(0, self.R)
        else:
            y_k = None
        return y_k

    def distance(self, tracker, target):
        d_k = np.linalg.norm(tracker - target)

        return d_k