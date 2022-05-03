"""

Author: Francisco Branco
Created: 22/12/2020

"""

# Path Following module for vehicle control

import numpy as np
import utils
import control as ct
from math import pi

class Lapierre(ct.NonlinearIOSystem):
    def __init__(self, some_path, name="Lapierre", gamma=1, k1=1, k2=1, k_delta=1, theta_a=pi/4):
        self.delta_func = utils.delta_func(k_delta, theta_a)
        self.path = some_path
        self.name = name

        params = {
            "gamma": gamma,
            "k1": k1,
            "k2": k2,
            "k_delta": k_delta,
            "theta_a": theta_a}

        super().__init__(
            updfcn=self.pf_update,
            outfcn=self.pf_output,
            params=params,
            name=name,
            inputs=("theta_m", "velocity", "velocity_dot", "vehicle_x", "vehicle_y"),
            outputs=("u", "s"),
            states=("s", "s1", "y1"))

    def calculate_theta(self, x, u):
        s = x[0]
        theta_m = u[0]
        theta_c = self.path.get_theta_c(s)

        return utils.angle_difference(theta_c, theta_m)

    def s1_dot(self, x ,u, params):
        s = x[0]
        s1, y1 = self.distance_geometry(x, u)
        #print("s1 = " + str(s1) + " y1 = " + str(y1))
        theta = self.calculate_theta(x, u)
        v = u[1]
        Cc = self.path.get_curvature(s)

        return (-1) * (1 - Cc * y1) + v * np.cos(theta)

    def y1_dot(self, x, u, params):
        s = x[0]
        s1, _ = self.distance_geometry(x, u)
        theta = self.calculate_theta(x, u)
        Cc = self.path.get_curvature(s)
        v = u[1]
        s_dot = self.s_dot_law(x, u, params)

        return (-1) * Cc * s_dot * s1 + v * np.sin(theta)

    def s_dot_law(self, x, u, params):
        s = x[0]
        s1, _ = self.distance_geometry(x, u)
        theta = self.calculate_theta(x, u)
        v = u[1]
        s_dot = v * np.cos(theta) + params.get("k1") * s1
        #print("s_dot = " + str(s_dot))
        if s <= 0 and s_dot < 0:
            return 0
        elif s >= 1 and s_dot > 0:
            return 0
        else:
            return s_dot

    def theta_dot_law(self, x, u, params):
        _, y1 = self.distance_geometry(x, u)
        theta = self.calculate_theta(x, u)
        v = u[1]
        v_dot = u[2]
        delta = self.delta_func.output(y1, v)
        delta_dot = self.delta_func.dot(y1, v, self.y1_dot(x, u, params), v_dot)

        dividing_term = theta - delta
        """
        if dividing_term == 0:
            dividing_term = 0.0000001
        """

        return delta_dot - params.get("gamma") * y1 * v * (np.sin(theta) - np.sin(delta)) / (dividing_term) - params.get("k2") * (theta - delta)

    def pf_update(self, t, x, u, params={}):
        #print("s = " + str(x[0]))
        s_update = self.s_dot_law(x, u, params)
        s1_update = self.s1_dot(x, u, params)
        y1_update = self.y1_dot(x, u, params)

        #print("s_update = " + str(s_update))

        return (s_update, s1_update, y1_update)
    
    def pf_output(self, t, x, u, params={}):
        s = x[0]
        if s < 0:
            s = 0
        elif s > 1:
            s = 1
        #print(str(self.name) + " s = " + str(s))
        return (self.theta_dot_law(x, u, params) + self.path.get_curvature(s) * self.s_dot_law(x, u, params), s)

    def distance_geometry(self, x, u) -> (float, float):
        s = x[0]
        vehicle_x = u[3]
        vehicle_y = u[4]
        rabbit_x, rabbit_y = self.path.get_xy(s)
        s1, y1 = utils.rotation_matrix((-1) * self.path.get_theta_c(s)).dot(np.array([vehicle_x - rabbit_x, vehicle_y - rabbit_y]))

        return s1, y1
        