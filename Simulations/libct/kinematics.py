"""

Author: Francisco Branco
Created: 22/12/2020

"""

import numpy as np
import control as ct
import utils


class Kinematics(ct.NonlinearIOSystem):
    
    def __init__(self, name="AUV"):
        super().__init__(
            self.auv_update,
            self.auv_output,
            name=name,
            inputs=("u", "velocity", "velocity_dot"),
            outputs=("x_out", "y_out", "theta_m_out"),
            states=("x", "y", "theta_m"))
        
    def auv_update(self, t, x, u, params={}):
        x[2] = utils.angle_wrapper(x[2])
        x_update = self.x_dot(x, u)
        y_update = self.y_dot(x, u)
        theta_m_update = self.theta_m_dot(x, u)

        #print(u[1])

        return (x_update, y_update, theta_m_update)

    def auv_output(self, t, x, u, params={}):
        x_output = x[0]
        y_output = x[1]
        theta_m_output = utils.angle_wrapper(x[2])
        
        return (x_output, y_output, theta_m_output)
    
    def x_dot(self, x, u):
        v = u[1]
        theta_m = x[2]
        #print("velocity = " + str(v))

        return v * np.cos(theta_m)

    def y_dot(self, x, u):
        v = u[1]
        theta_m = x[2]

        return v * np.sin(theta_m)

    def theta_m_dot(self, x, u):
        return u[0]