"""

Author: Francisco Branco
Created: 05/05/2021

"""


import numpy as np
import utils
from scipy.integrate import odeint as oi
from typing import Tuple


class Kinematics:
    def __init__(self, name="AUV", saturate=0, state_history=False, dt=1):
        self.name = name
        self.state_history = state_history
        self.dt = dt
        self.saturate = saturate

        if self.state_history:
            self.past_state = {"x": [], "y": [], "theta_m": []}

        self.state = {"x": 0, "y": 0, "theta_m": 0}
        self.inputs= {"u": 0, "velocity": 0, "velocity_dot": 0}
        
    def auv_update(self, inputs=None, dt=None):
        if dt is None:
            dt = self.dt
        if inputs is not None:
            for i in self.inputs.keys():
                self.inputs[i] = inputs[i]
        
        x_update = self.x_dot()
        y_update = self.y_dot()
        theta_m_update = self.theta_m_dot()

        self.state["x"] = self.state["x"] + x_update * dt
        self.state["y"] = self.state["y"] + y_update * dt

        self.state["theta_m"] = self.state["theta_m"] + theta_m_update * dt
        self.state["theta_m"] = utils.angle_wrapper(self.state["theta_m"])

        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])

    def auv_output(self) -> Tuple[dict, dict]:
        return (self.state["x"], self.state["y"], self.state["theta_m"])

    def inputs_outputs(self) -> Tuple[dict, dict]:
        outputs = self.state.copy()
        outputs["x_dot"] = self.x_dot()
        outputs["y_dot"] = self.y_dot()

        return self.inputs.copy(), outputs

    def set_initial_conditions(self, x=float, y=float, theta_m=float):
        self.state["x"] = x
        self.state["y"] = y
        self.state["theta_m"] = theta_m
    
    def x_dot(self) -> float:
        v = self.inputs["velocity"]
        theta_m = self.state["theta_m"]

        return v * np.cos(theta_m)

    def y_dot(self) -> float:
        v = self.inputs["velocity"]
        theta_m = self.state["theta_m"]

        return v * np.sin(theta_m)

    def theta_m_dot(self) -> float:
        if self.saturate == 0:
            theta_m_dot = self.inputs["u"]
        else:
            theta_m_dot = self.saturate * np.tanh(self.inputs["u"])
        return theta_m_dot

    