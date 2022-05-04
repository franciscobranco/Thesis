"""

Author: Francisco Branco
Created: 10/05/2020

"""


import numpy as np
from math import pi

import lib.kinematics as kn
import lib.pathfollowing as pf
import lib.movingpathfollowing as mpf
import lib.extendedkalmanfilter as ekf


class ASVMPFOnAUVTargetPursuit:
    def __init__(self, path_target, path_tracker, target_speed=1, tracker_speed=1, pf_params=None, ekf_params=None, time_halted=50, history=False, dt=1):
        self.dt = dt
        self.path_target = path_target
        self.path_tracker = path_tracker
        self.target_speed = target_speed
        self.tracker_speed = tracker_speed
        self.pf_prarams = pf_params
        self.ekf_params = ekf_params
        self.time_halted = time_halted

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
                "velocity_target": [],
                "velocity_tracker": [],
                "x_ekf": [],
                "y_ekf": [],
                "theta_ekf": [],
                "velocity_ekf": [],
                "theta_dot_ekf": [],
                "range": []
            }
        else:
            state_history = False


        self.kine_target = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target = pf.Lapierre(some_path=path_target, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        
        self.mpf_control_tracker = mpf.MovingPathFollowingTest(target_velocity="Multiple", saturate=0, state_history=state_history, dt=dt)
        self.pf_control_tracker = pf.Lapierre(some_path=path_tracker, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.ekf_tracker = ekf.ExtendedKalmanFilter(F_matrix=ekf_params["F_matrix"], Q_matrix=ekf_params["Q_matrix"], R_matrix=ekf_params["R_matrix"], dt=dt)
        self.rms_tracker = ekf.RangeMeasureSimulation(R=ekf_params["R_matrix"], sampling_period=2)

    def update(self, t):
        # Get dictionaries setup
        inputs_kine_target, outputs_kine_target = self.kine_target.inputs_outputs()
        inputs_pf_target, outputs_pf_target = self.pf_control_target.inputs_outputs()

        inputs_mpf_tracker, outputs_mpf_tracker = self.mpf_control_tracker.inputs_outputs()
        inputs_pf_tracker, outputs_pf_tracker = self.pf_control_tracker.inputs_outputs()
        inputs_ekf_tracker, outputs_ekf_tracker = self.ekf_tracker.inputs_outputs()


        # EKF Prediction and Update
        inputs_ekf_tracker["tracker_x0"] = outputs_mpf_tracker["x"]
        inputs_ekf_tracker["tracker_y0"] = outputs_mpf_tracker["y"]
        tracker_pos = np.array([outputs_mpf_tracker["x"], outputs_mpf_tracker["y"]])
        target_pos = np.array([outputs_kine_target["x"], outputs_kine_target["y"]])
        y_k = self.rms_tracker.measurement(t, tracker_pos, target_pos)
        if t < 150 or t > 200:
            inputs_ekf_tracker["range0"] = y_k
        else:
            inputs_ekf_tracker["range0"] = None

        self.ekf_tracker.ekf_update(inputs_ekf_tracker, t)
        inputs_ekf_tracker, outputs_ekf_tracker = self.ekf_tracker.inputs_outputs()


        # Connection of variables
        # AUV Target
        inputs_kine_target["u"] = outputs_pf_target["u"]
        if t < self.time_halted:
            inputs_kine_target["velocity"] = 0.0
            inputs_pf_target["velocity"] = 0.0
        else:
            inputs_kine_target["velocity"] = self.target_speed
            inputs_pf_target["velocity"] = self.target_speed
        inputs_kine_target["velocity_dot"] = 0
        
        inputs_pf_target["x"] = outputs_kine_target["x"]
        inputs_pf_target["y"] = outputs_kine_target["y"]
        inputs_pf_target["theta_m"] = outputs_kine_target["theta_m"]
        
        inputs_pf_target["velocity_dot"] = 0


        # ASV
        if t < self.time_halted:
            inputs_mpf_tracker["target_x"] = -4.
            inputs_mpf_tracker["target_y"] = -10.
            inputs_mpf_tracker["target_x_dot"] = 0.
            inputs_mpf_tracker["target_y_dot"] = 0.
        else:
            inputs_mpf_tracker["target_x"] = outputs_ekf_tracker["x"]
            inputs_mpf_tracker["target_y"] = outputs_ekf_tracker["y"]
            inputs_mpf_tracker["target_x_dot"] = outputs_ekf_tracker["x_dot"]
            inputs_mpf_tracker["target_y_dot"] = outputs_ekf_tracker["y_dot"]
        
        #inputs_mpf_tracker["target_x"] = outputs_ekf_tracker["x"]
        #inputs_mpf_tracker["target_y"] = outputs_ekf_tracker["y"]
        #inputs_mpf_tracker["target_x_dot"] = outputs_ekf_tracker["x_dot"]
        #inputs_mpf_tracker["target_y_dot"] = outputs_ekf_tracker["y_dot"]
        inputs_mpf_tracker["target_yaw"] = 0#outputs_ekf_tracker["theta"]
        inputs_mpf_tracker["target_u"] = 0#outputs_ekf_tracker["theta_dot"]
        #inputs_mpf_tracker["target_velocity"] = 0
        
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
        if t < self.time_halted:
            outputs["velocity_target"] = 0
        else:
            outputs["velocity_target"] = self.target_speed
        outputs["velocity_tracker"] = outputs_mpf_tracker["velocity"]
        outputs["x_ekf"] = outputs_ekf_tracker["x"]
        outputs["y_ekf"] = outputs_ekf_tracker["y"]
        outputs["theta_ekf"] = outputs_ekf_tracker["theta"]
        #print(np.power(outputs_ekf_tracker["x_dot"], 2))
        outputs["velocity_ekf"] = np.sqrt(np.power(outputs_ekf_tracker["x_dot"], 2) + np.power(outputs_ekf_tracker["y_dot"], 2))
        outputs["theta_dot_ekf"] = outputs_ekf_tracker["theta_dot"]
        outputs["range"] = y_k

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

        ry = self.mpf_control_tracker.reference_yaw(init_cond["theta_m_follower"], init_cond["theta_m_target"])
        self.mpf_control_tracker.set_initial_conditions(init_cond["x_follower"], init_cond["y_follower"], ry)
        self.pf_control_tracker.set_initial_conditions(init_cond["s_follower"])
        ic_ekf = {
            "x": init_cond["x_target"],
            "y": init_cond["y_target"],
            "x_dot": 0,
            "y_dot": 0
        }
        self.ekf_tracker.set_initial_conditions(ic_ekf)

    def past_values(self):
        return (
            self.past_outputs,
            self.kine_target.past_state,
            self.pf_control_target.past_state,
            self.mpf_control_tracker.past_state,
            self.pf_control_tracker.past_state,
            self.ekf_tracker.past_state
        )