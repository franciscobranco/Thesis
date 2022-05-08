"""

Author: Francisco Branco
Created: 09/05/2021

"""


import numpy as np
from math import pi
import utils
from scipy.integrate import odeint as oi
from typing import Tuple


class Lapierre:
    def __init__(self, some_path, gamma=1, k1=1, k2=1, k_delta=1, theta_a=pi/4, state_history=False, dt=1):
        self.delta_func = utils.delta_func(k_delta, theta_a)
        self.path = some_path
        self.state_history = state_history
        self.dt = dt
        self.laps = 0

        self.params = {
            "gamma": gamma,
            "k1": k1,
            "k2": k2,
            "k_delta": k_delta,
            "theta_a": theta_a
            }

        if self.state_history:
            self.past_state = {
                "s": [], 
                "s1": [], 
                "y1": [], 
                "s1_geo": [], 
                "y1_geo": []
                }

        self.state = {"s": 0, "s1": 0, "y1": 0}
        self.inputs= {"x": 0, "y": 0, "theta_m": 0, "velocity": 0, "velocity_dot": 0}

    def pf_update(self, inputs=None, dt=None):
        if dt is None:
            dt = self.dt
        if inputs is not None:
            for key in self.inputs.keys():
                self.inputs[key] = inputs[key]
        
        norm = self.path.total_distance

        s_update = self.s_dot_law() / norm
        s1_update = self.s1_dot()
        y1_update = self.y1_dot()

        self.state["s"] = self.state["s"] + s_update * dt
        if self.state["s"] > 1:
            self.state["s"] = self.state["s"] - 1
            self.laps = self.laps + 1
            #self.state["s"] = 1
        elif self.state["s"] < 0:
            self.state["s"] = self.state["s"] + 1
            # self.state["s"] = 0

        self.state["s1"] = self.state["s1"] + s1_update * dt
        self.state["y1"] = self.state["y1"] + y1_update * dt
        
        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])
            s1, y1 = self.distance_geometry()
            self.past_state["s1_geo"].append(s1)
            self.past_state["y1_geo"].append(y1)

    def pf_output(self) -> Tuple[float, float]:
        s = self.state["s"]
        theta_dot_law = self.theta_dot_law()
        Cc = self.path.get_curvature(s)
        s_dot_law = self.s_dot_law()
        u = theta_dot_law + (Cc * s_dot_law)
        s1, y1 = self.distance_geometry()
        #print(u, s, s1, y1, theta_dot_law)
        return (u, s, s1, y1)

    def inputs_outputs(self) -> Tuple[dict, dict]:
        u, s, s1, y1 = self.pf_output()
        outputs = {"u": u, "s": s, "s1": s1, "y1": y1}
        
        return (self.inputs.copy(), outputs)

    def set_initial_conditions(self, s=float):
        self.state["s"] = s
        s1, y1 = self.distance_geometry()
        self.state["s1"] = s1
        self.state["y1"] = y1

    def calculate_theta(self) -> float:
        s = self.state["s"]
        theta_m = self.inputs["theta_m"]
        theta_c = self.path.get_theta_c(s)

        return utils.angle_difference(theta_m, theta_c)

    def s1_dot(self) -> float:
        s = self.state["s"]
        _, y1 = self.distance_geometry()
        theta = self.calculate_theta()
        v = self.inputs["velocity"]
        Cc = self.path.get_curvature(s)
        s_dot = self.s_dot_law()
        
        s1_dot = ((-1) * s_dot * (1 - Cc * y1)) + (v * np.cos(theta))

        return s1_dot

    def y1_dot(self) -> float:
        s = self.state["s"]
        s1, _ = self.distance_geometry()
        theta = self.calculate_theta()
        Cc = self.path.get_curvature(s)
        v = self.inputs["velocity"]
        s_dot = self.s_dot_law()

        # print("s1 ", s1, " s_dot ", s_dot)

        y1_dot = ((-1) * Cc * s_dot * s1) + (v * np.sin(theta))

        return y1_dot

    def s_dot_law(self) -> float:
        s1, _ = self.distance_geometry()
        theta = self.calculate_theta()
        v = self.inputs["velocity"]
        s_dot = (v * np.cos(theta)) + (self.params["k1"] * s1)

        return s_dot

    def theta_dot_law(self) -> float:
        _, y1 = self.distance_geometry()
        theta = self.calculate_theta()
        v = self.inputs["velocity"]
        v_dot = self.inputs["velocity_dot"]
        delta = self.delta_func.output(y1, v)
        y1_dot = self.y1_dot()
        delta_dot = self.delta_func.dot(y1, v, y1_dot, v_dot)

        dividing_term = theta - delta
        
        if dividing_term == 0:
            theta_dot_law = delta_dot - (self.params["gamma"] * y1 * v)
        else:
            theta_dot_law = delta_dot - (self.params["gamma"] * y1 * v * ((np.sin(theta) - np.sin(delta)) / dividing_term)) - (self.params["k2"] * (theta - delta))

        # print("y1 ", y1, " y1_dot ", y1_dot)
        # print("delta_dot ", delta_dot)
        # print("theta_dot_law ", theta_dot_law)

        return theta_dot_law

    def distance_geometry(self) -> Tuple[float, float]:
        s = self.state["s"]
        vehicle_x = self.inputs["x"]
        vehicle_y = self.inputs["y"]
        rabbit_x, rabbit_y = self.path.get_xy(s)

        res = np.matmul(utils.rotation_matrix((-1) * self.path.get_theta_c(s)), (np.array([vehicle_x - rabbit_x, vehicle_y - rabbit_y])))
        
        s1 = res[0]
        y1 = res[1]

        return (s1, y1)