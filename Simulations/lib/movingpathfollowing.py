"""

Author: Francisco Branco
Created: 22/09/2021
Description: Moving Path Following module

"""


import utils
from math import pi
import numpy as np


class MovingPathFollowing:
    def __init__(self, target_velocity="Single", follower_velocity="Single", saturate=1, rotating_path=False, state_history=False, dt=1):
        self.state_history = state_history
        self.dt = dt
        self.saturate = saturate
        self.rotating_path = rotating_path
        self.target_velocity = target_velocity
        self.follower_velocity = follower_velocity

        self.state = {
            "x": 0,
            "y": 0,
            "theta_m_ref": 0,
            "velocity": 0
        }

        self.inputs = {
            "target_x": 0,
            "target_y": 0,
            "target_yaw": 0,
            "target_u": 0,
            "follower_u": 0
        }
        if target_velocity == "Single":
            self.inputs["target_velocity"] = 0
        elif target_velocity == "Multiple":
            self.inputs["target_x_dot"] = 0
            self.inputs["target_y_dot"] = 0
        if follower_velocity == "Single":
            self.inputs["follower_velocity"] = 0
        elif target_velocity == "Multiple":
            self.inputs["follower_x_dot"] = 0
            self.inputs["follower_y_dot"] = 0

        if state_history:
            self.past_state = {
                "x": [],
                "y": [],
                "theta_m_ref": [],
                "velocity": []
            }

    def set_initial_conditions(self, x, y, theta_m_ref):
        self.state["x"] = x
        self.state["y"] = y
        self.state["theta_m_ref"] = theta_m_ref

    def inputs_outputs(self):
        outputs = {}
        point_ref = self.point_in_reference(self.state["x"], self.state["y"], self.inputs["target_yaw"], self.inputs["target_x"], self.inputs["target_y"])
        outputs["x"] = self.state["x"]
        outputs["y"] = self.state["y"]
        outputs["theta_m"] = self.inertial_yaw()
        outputs["x_ref"] = point_ref[0]
        outputs["y_ref"] = point_ref[1]
        outputs["theta_m_ref"] = self.state["theta_m_ref"]
        outputs["velocity"] = self.state["velocity"]

        return self.inputs.copy(), outputs

    def mpf_update(self, inputs, dt=None):
        if dt is None:
            dt = self.dt

        for key in inputs.keys():
            self.inputs[key] = inputs[key]

        
        follower_velocity = self.inertial_guidance_law()
        if self.saturate == 0:
            theta_m_dot = self.inputs["follower_u"]
        else:
            theta_m_dot = np.clip(self.inputs["follower_u"], (-1) * np.abs(self.saturate), np.abs(self.saturate))
        
        self.state["x"] = self.state["x"] + follower_velocity[0] * dt
        self.state["y"] = self.state["y"] + follower_velocity[1] * dt
        self.state["theta_m_ref"] = utils.angle_wrapper(self.state["theta_m_ref"] + theta_m_dot * dt)
        self.state["velocity"] = np.linalg.norm(follower_velocity)

        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])

    def follower_x_dot(self):
        if self.follower_velocity == "Single":
            theta_m = self.inertial_yaw()
            x_dot = self.inputs["follower_velocity"] * np.cos(theta_m)
        elif self.follower_velocity == "Multiple":
            x_dot = self.inputs["follower_x_dot"]

        return x_dot

    def follower_y_dot(self):
        if self.follower_velocity == "Single":
            theta_m = self.inertial_yaw()
            y_dot = self.inputs["follower_velocity"] * np.sin(theta_m)
        elif self.follower_velocity == "Multiple":
            y_dot = self.inputs["follower_y_dot"]
        
        return y_dot

    def target_x_dot(self):
        if self.target_velocity == "Single":
            x_dot = self.inputs["target_velocity"] * np.cos(self.inputs["target_yaw"])
        elif self.target_velocity == "Multiple":
            x_dot = self.inputs["target_x_dot"]

        return x_dot

    def target_y_dot(self):
        if self.target_velocity == "Single":
            y_dot = self.inputs["target_velocity"] * np.sin(self.inputs["target_yaw"])
        elif self.target_velocity == "Multiple":
            y_dot = self.inputs["target_y_dot"]

        return y_dot

    def reference_yaw(self, follower_yaw, target_yaw):
        if self.rotating_path:
            ry = utils.angle_wrapper(follower_yaw - target_yaw)
        else:
            ry = follower_yaw
        
        return ry
    
    def point_in_reference(self, follower_x, follower_y, target_yaw, target_x, target_y):
        follower = np.zeros((2,))
        follower[0] = follower_x
        follower[1] = follower_y

        target = np.zeros((2,))
        target[0] = target_x
        target[1] = target_y

        if self.rotating_path:
            pir = np.matmul(np.linalg.inv(utils.rotation_matrix(target_yaw)), follower - target)
        else:
            pir = follower - target

        return pir

    def inertial_yaw(self):
        if self.rotating_path:
            iy = utils.angle_wrapper(self.state["theta_m_ref"] + self.inputs["target_yaw"])
        else:
            iy = self.state["theta_m_ref"]

        return iy

    """
    def point_in_inertial(self):
        point_reference = np.zeros((2,))
        point_reference[0] = self.state["x"] - self.inputs["target_x"]
        point_reference[1] = self.state["y"] - self.inputs["target_y"]

        target = np.zeros((2,))
        target[0] = self.inputs["target_x"]
        target[1] = self.inputs["target_y"]

        pii = target + utils.rotation_matrix(self.inputs["target_yaw"]).dot(point_reference)
        return pii
    """

    def inertial_guidance_law(self):
        target_vel = np.zeros((2,))
        target_vel[0] = self.target_x_dot()
        target_vel[1] = self.target_y_dot()

        point_ref = self.point_in_reference(self.state["x"], self.state["y"], self.inputs["target_yaw"], self.inputs["target_x"], self.inputs["target_y"])

        follower_vel = np.zeros((2,))
        follower_vel[0] = self.follower_x_dot()
        follower_vel[1] = self.follower_y_dot()

        if self.rotating_path:
            u_vec = np.zeros((2,))
            if self.saturate != 0:
                u_vec[0] = (-1) * np.clip(self.inputs["target_u"], (-1) * np.abs(self.saturate), np.abs(self.saturate)) * point_ref[1]
                u_vec[1] = np.clip(self.inputs["target_u"], (-1) * np.abs(self.saturate), np.abs(self.saturate)) * point_ref[0]
            else:
                u_vec[0] = (-1) * self.inputs["target_u"] * point_ref[1]
                u_vec[1] = self.inputs["target_u"] * point_ref[0]
            igl = target_vel + utils.rotation_matrix(self.inputs["target_yaw"]).dot(u_vec) + np.matmul(utils.rotation_matrix(self.inputs["target_yaw"]), follower_vel)
        else:
            igl = target_vel + follower_vel
        
        return igl


"""
class MovingPathFollowing:
    def __init__(self, target_velocity="Single", follower_velocity="Single", saturate=1, state_history=False, dt=1):
        self.state_history = state_history
        self.dt = dt
        self.saturate = saturate
        self.target_velocity = target_velocity
        self.follower_velocity = follower_velocity

        self.state = {
            "x": 0,
            "y": 0,
            "theta_m_ref": 0,
        }

        self.inputs = {
            "target_x": 0,
            "target_y": 0,
            "target_yaw": 0,
            "target_u": 0,
            "follower_u": 0,
        }
        if target_velocity == "Single":
            self.inputs["target_velocity"] = 0
        elif target_velocity == "Multiple":
            self.inputs["target_x_dot"] = 0
            self.inputs["target_y_dot"] = 0
        if follower_velocity == "Single":
            self.inputs["follower_velocity"] = 0
        elif target_velocity == "Multiple":
            self.inputs["follower_x_dot"] = 0
            self.inputs["follower_y_dot"] = 0

        if state_history:
            self.past_state = {
                "x": [],
                "y": [],
                "theta_m_ref": []
            }

    def set_initial_conditions(self, x, y, theta_m_ref):
        self.state["x"] = x
        self.state["y"] = y
        self.state["theta_m_ref"] = theta_m_ref

    def inputs_outputs(self):
        outputs = {}
        point_ref = self.point_in_reference(self.state["x"], self.state["y"], self.inputs["target_yaw"], self.inputs["target_x"], self.inputs["target_y"])
        outputs["x"] = self.state["x"]
        outputs["y"] = self.state["y"]
        outputs["theta_m"] = self.inertial_yaw()
        outputs["x_ref"] = point_ref[0]
        outputs["y_ref"] = point_ref[1]
        outputs["theta_m_ref"] = self.state["theta_m_ref"]

        return self.inputs.copy(), outputs

    def mpf_update(self, inputs, dt=None):
        if dt is None:
            dt = self.dt

        for key in inputs.keys():
            self.inputs[key] = inputs[key]

        
        follower_velocity = self.inertial_guidance_law()
        if self.saturate == 0:
            theta_m_dot = self.inputs["follower_u"]
        else:
            theta_m_dot = self.saturate * np.tanh(self.inputs["follower_u"])
        
        self.state["x"] = self.state["x"] + follower_velocity[0] * dt
        self.state["y"] = self.state["y"] + follower_velocity[1] * dt
        self.state["theta_m_ref"] = utils.angle_wrapper(self.state["theta_m_ref"] + theta_m_dot * dt)

        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])

    def follower_x_dot(self):
        if self.follower_velocity == "Single":
            x_dot = self.inputs["follower_velocity"] * np.cos(self.state["theta_m_ref"])
        elif self.follower_velocity == "Multiple":
            x_dot = self.inputs["follower_x_dot"]

        return x_dot

    def follower_y_dot(self):
        if self.follower_velocity == "Single":
            y_dot = self.inputs["follower_velocity"] * np.sin(self.state["theta_m_ref"])
        elif self.follower_velocity == "Multiple":
            y_dot = self.inputs["follower_y_dot"]
        
        return y_dot

    def target_x_dot(self):
        if self.target_velocity == "Single":
            x_dot = self.inputs["target_velocity"] * np.cos(self.inputs["target_yaw"])
        elif self.target_velocity == "Multiple":
            x_dot = self.inputs["target_x_dot"]

        return x_dot

    def target_y_dot(self):
        if self.target_velocity == "Single":
            y_dot = self.inputs["target_velocity"] * np.sin(self.inputs["target_yaw"])
        elif self.target_velocity == "Multiple":
            y_dot = self.inputs["target_y_dot"]

        return y_dot

    def reference_yaw(self, follower_yaw, target_yaw):
        ry = utils.angle_wrapper(follower_yaw - target_yaw)
        return ry
    
    def point_in_reference(self, follower_x, follower_y, target_yaw, target_x, target_y):
        follower = np.zeros((2,))
        follower[0] = follower_x
        follower[1] = follower_y

        target = np.zeros((2,))
        target[0] = target_x
        target[1] = target_y

        pir = np.linalg.inv(utils.rotation_matrix(target_yaw)).dot(follower - target)
        return pir

    def inertial_yaw(self):
        iy = utils.angle_wrapper(self.state["theta_m_ref"] + self.inputs["target_yaw"])
        return iy

    
    # def point_in_inertial(self):
    #     point_reference = np.zeros((2,))
    #     point_reference[0] = self.state["x"] - self.inputs["target_x"]
    #     point_reference[1] = self.state["y"] - self.inputs["target_y"]

    #     target = np.zeros((2,))
    #     target[0] = self.inputs["target_x"]
    #     target[1] = self.inputs["target_y"]

    #     pii = target + utils.rotation_matrix(self.inputs["target_yaw"]).dot(point_reference)
    #     return pii
    

    def inertial_guidance_law(self):
        target_vel = np.zeros((2,))
        target_vel[0] = self.target_x_dot()
        target_vel[1] = self.target_y_dot()

        point_ref = self.point_in_reference(self.state["x"], self.state["y"], self.inputs["target_yaw"], self.inputs["target_x"], self.inputs["target_y"])

        u_vec = np.zeros((2,))
        if 
            if self.saturate != 0:
                u_vec[0] = (-1) * np.clip(self.inputs["target_u"], (-1) * np.abs(self.saturate), np.abs(self.saturate)) * point_ref[1]
                u_vec[1] = np.clip(self.inputs["target_u"], (-1) * np.abs(self.saturate), np.abs(self.saturate)) * point_ref[0]
            else:
                u_vec[0] = (-1) * self.inputs["target_u"] * point_ref[1]
                u_vec[1] = self.inputs["target_u"] * point_ref[0]

        follower_vel = np.zeros((2,))
        follower_vel[0] = self.follower_x_dot()
        follower_vel[1] = self.follower_y_dot()

        igl = target_vel + utils.rotation_matrix(self.inputs["target_yaw"]).dot(u_vec) + utils.rotation_matrix(self.inputs["target_yaw"]).dot(follower_vel)
        return igl

    """