"""

Author: Francisco Branco
Created: 10/05/2020

"""


import numpy as np
from math import pi

import lib.kinematics as kn
import lib.pathfollowing as pf
import lib.cooperativepathfollowing as cpf
import lib.movingpathfollowing as mpf


class DoubleAUVMPFETC:
    def __init__(self, path_target, path_tracker, target_speed=1, pf_params=None, tracker_speed=1, history=False, dt=1):
        self.dt = dt
        self.path_target = path_target
        self.path_tracker = path_tracker
        self.pf_prarams = pf_params
        self.target_speed = target_speed
        self.tracker_speed = tracker_speed
        
        #self.inputs = {"velocity": 0, "velocity_dot": 0}
        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_outputs = {
                "x_target": [],
                "y_target": [],
                "theta_m_target": [],
                "s_target": [],
                "u_target": [],
                "x_tracker": [],
                "y_tracker": [],
                "theta_m_tracker": [],
                "s_tracker": [],
                "u_tracker": [],
            }
        else:
            state_history = False

        self.kine_target = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target = pf.Lapierre(some_path=path_target, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)

        self.mpf_control_tracker = mpf.MovingPathFollowing(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_tracker = pf.Lapierre(some_path=path_tracker, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        
    def update(self, t):
        
        # Get dictionaries setup
        inputs_kine_target, outputs_kine_target = self.kine_target.inputs_outputs()
        inputs_pf_target, outputs_pf_target = self.pf_control_target.inputs_outputs()

        inputs_mpf_tracker, outputs_mpf_tracker = self.mpf_control_tracker.inputs_outputs()
        inputs_pf_tracker, outputs_pf_tracker = self.pf_control_tracker.inputs_outputs()



        # Connection of variables
        # Target AUV
        inputs_kine_target["u"] = outputs_pf_target["u"]
        inputs_kine_target["velocity"] = self.target_speed
        inputs_kine_target["velocity_dot"] = 0
        
        inputs_pf_target["x"] = outputs_kine_target["x"]
        inputs_pf_target["y"] = outputs_kine_target["y"]
        inputs_pf_target["theta_m"] = outputs_kine_target["theta_m"]
        inputs_pf_target["velocity"] = self.target_speed
        inputs_pf_target["velocity_dot"] = 0

        # Tracker ASV
        inputs_mpf_tracker["target_x"] = outputs_kine_target["x"]
        inputs_mpf_tracker["target_y"] = outputs_kine_target["y"]
        inputs_mpf_tracker["target_yaw"] = outputs_kine_target["theta_m"]
        inputs_mpf_tracker["target_u"] = outputs_pf_target["u"]
        inputs_mpf_tracker["target_velocity"] = self.target_speed
        inputs_mpf_tracker["follower_u"] = outputs_pf_tracker["u"]
        inputs_mpf_tracker["follower_velocity"] = self.tracker_speed
        
        inputs_pf_tracker["x"] = outputs_mpf_tracker["x_ref"]
        inputs_pf_tracker["y"] = outputs_mpf_tracker["y_ref"]
        inputs_pf_tracker["theta_m"] = outputs_mpf_tracker["theta_m_ref"]
        inputs_pf_tracker["velocity"] = self.tracker_speed
        inputs_pf_tracker["velocity_dot"] = 0


        outputs = {}
        outputs["x_target"] = outputs_kine_target["x"]
        outputs["y_target"] = outputs_kine_target["y"]
        outputs["theta_m_target"] = outputs_kine_target["theta_m"]
        outputs["s_target"] = outputs_pf_target["s"]
        outputs["u_target"] = outputs_pf_target["u"]
        outputs["x_tracker"] = outputs_mpf_tracker["x"]
        outputs["y_tracker"] = outputs_mpf_tracker["y"]
        outputs["theta_m_tracker"] = outputs_mpf_tracker["theta_m"]
        outputs["s_tracker"] = outputs_pf_tracker["s"]
        outputs["u_tracker"] = outputs_pf_tracker["u"]

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine_target.auv_update(inputs_kine_target, dt=self.dt)
        self.pf_control_target.pf_update(inputs_pf_target, dt=self.dt)
        self.mpf_control_tracker.mpf_update(inputs_mpf_tracker, dt=self.dt)
        self.pf_control_tracker.pf_update(inputs_pf_tracker, dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine_target.set_initial_conditions(init_cond["x_target"], init_cond["y_target"], init_cond["theta_m_target"])
        self.pf_control_target.set_initial_conditions(init_cond["s_target"])

        #pir = self.mpf_control_follower.point_in_reference(init_cond["x1"], init_cond["y1"], init_cond["x0"], init_cond["y0"], init_cond["theta_m0"])
        ry = self.mpf_control_tracker.reference_yaw(init_cond["theta_m_tracker"], init_cond["theta_m_target"])
        self.mpf_control_tracker.set_initial_conditions(init_cond["x_tracker"], init_cond["y_tracker"], ry)

    def past_values(self):
        return (self.past_outputs, self.kine_target.past_state, self.pf_control_target.past_state, self.mpf_control_tracker.past_state, self.pf_control_tracker.past_state)