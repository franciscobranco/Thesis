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


class DoubleAUVMPFETCTest:
    def __init__(self, path0, path1, pf_params=None, cpf_params=None, etc_type="Time", factor=10, history=False, dt=1):
        self.dt = dt
        self.factor = factor
        self.path0 = path0
        self.path1 = path1
        self.pf_prarams = pf_params
        self.cpf_params = cpf_params
        
        #self.inputs = {"velocity": 0, "velocity_dot": 0}
        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_outputs = {
                "x0": [],
                "y0": [],
                "theta_m0": [],
                "s0": [],
                "u0": [],
                "x1": [],
                "y1": [],
                "theta_m1": [],
                "s1": [],
                "u1": [],
                "velocity0": [],
                "velocity1": []
                }
        else:
            state_history = False

        A_matrix = np.array([[0, 1],[1, 0]])

        self.kine_target = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target = pf.Lapierre(some_path=path0, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        #self.cpf_control_target = cpf.CPFContinuousController(num_auv=2, id=0, k_csi=cpf_params["k_csi0"], params=cpf_params, A_matrix=A_matrix, state_history=state_history, dt=dt)
        self.cpf_control_target = cpf.CPFDiscreteControllerETC(num_auv=2, id=0, params=cpf_params, k_csi=cpf_params["k_csi0"], A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=dt)

        self.mpf_control_follower = mpf.MovingPathFollowingTest(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_follower = pf.Lapierre(some_path=path1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        #self.cpf_control_follower = cpf.CPFContinuousController(num_auv=2, id=1, k_csi=cpf_params["k_csi1"], params=cpf_params, A_matrix=A_matrix, state_history=state_history, dt=dt)
        self.cpf_control_follower = cpf.CPFDiscreteControllerETC(num_auv=2, id=1, params=cpf_params, k_csi=cpf_params["k_csi1"], A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=dt)
        
    def update(self, t):
        
        # Get dictionaries setup
        inputs_kine_target, outputs_kine_target = self.kine_target.inputs_outputs()
        inputs_pf_target, outputs_pf_target = self.pf_control_target.inputs_outputs()
        inputs_cpf_target, outputs_cpf_target = self.cpf_control_target.inputs_outputs()

        inputs_mpf_follower, outputs_mpf_follower = self.mpf_control_follower.inputs_outputs()
        inputs_pf_follower, outputs_pf_follower = self.pf_control_follower.inputs_outputs()
        inputs_cpf_follower, outputs_cpf_follower = self.cpf_control_follower.inputs_outputs()



        self.cpf_control_target.inputs["gamma0"] = outputs_pf_target["s"]
        #self.cpf_control_target.inputs["gamma1"] = (outputs_pf_follower["s"] + self.pf_control_follower.laps) / self.factor

        #self.cpf_control_follower.inputs["gamma0"] = outputs_pf_target["s"]
        self.cpf_control_follower.inputs["gamma1"] = (outputs_pf_follower["s"] + self.pf_control_follower.laps) / self.factor



        
        # Check if comms needs update
        broadcast0 = self.cpf_control_target.check_update(t)
        if broadcast0 != -1:
            self.cpf_control_follower.reset(broadcast0)

        broadcast1 = self.cpf_control_follower.check_update(t)
        if broadcast1 != -1:
            self.cpf_control_target.reset(broadcast1)
        





        _, outputs_cpf_target = self.cpf_control_target.inputs_outputs()
        _, outputs_cpf_follower = self.cpf_control_follower.inputs_outputs()



        # Connection of variables
        # First AUV
        inputs_kine_target["u"] = outputs_pf_target["u"]
        inputs_kine_target["velocity"] = outputs_cpf_target["velocity"]
        inputs_kine_target["velocity_dot"] = outputs_cpf_target["velocity_dot"]
        
        inputs_pf_target["x"] = outputs_kine_target["x"]
        inputs_pf_target["y"] = outputs_kine_target["y"]
        inputs_pf_target["theta_m"] = outputs_kine_target["theta_m"]
        inputs_pf_target["velocity"] = outputs_cpf_target["velocity"]
        inputs_pf_target["velocity_dot"] = outputs_cpf_target["velocity_dot"]

        #self.cpf_control_target.inputs["gamma0"] = outputs_pf_target[""]
        self.cpf_control_target.inputs["gamma0"] = outputs_pf_target["s"]

        # Second AUV
        inputs_mpf_follower["target_x"] = self.path0.get_x(outputs_pf_target["s"])
        inputs_mpf_follower["target_y"] = self.path0.get_y(outputs_pf_target["s"])
        inputs_mpf_follower["target_yaw"] = self.path0.get_theta_c(outputs_pf_target["s"])
        inputs_mpf_follower["target_u"] = 0.017
        inputs_mpf_follower["target_velocity"] = self.cpf_params["speed_profile0"]
        inputs_mpf_follower["follower_u"] = outputs_pf_follower["u"]
        inputs_mpf_follower["follower_velocity"] = outputs_cpf_follower["velocity"]
        
        inputs_pf_follower["x"] = outputs_mpf_follower["x_ref"]
        inputs_pf_follower["y"] = outputs_mpf_follower["y_ref"]
        inputs_pf_follower["theta_m"] = outputs_mpf_follower["theta_m_ref"]
        inputs_pf_follower["velocity"] = outputs_cpf_follower["velocity"]
        inputs_pf_follower["velocity_dot"] = outputs_cpf_follower["velocity_dot"]

        self.cpf_control_follower.inputs["gamma1"] = (outputs_pf_follower["s"] + self.pf_control_follower.laps) / self.factor


        outputs = {}
        outputs["x0"] = outputs_kine_target["x"]
        outputs["y0"] = outputs_kine_target["y"]
        outputs["theta_m0"] = outputs_kine_target["theta_m"]
        outputs["x1"] = outputs_mpf_follower["x"]
        outputs["y1"] = outputs_mpf_follower["y"]
        outputs["theta_m1"] = outputs_mpf_follower["theta_m"]
        outputs["s0"] = outputs_pf_target["s"]
        outputs["u0"] = outputs_pf_target["u"]
        outputs["s1"] = outputs_pf_follower["s"]
        outputs["u1"] = outputs_pf_follower["u"]
        outputs["velocity0"] = outputs_cpf_target["velocity"]
        outputs["velocity1"] = outputs_cpf_follower["velocity"]

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine_target.auv_update(inputs_kine_target, dt=self.dt)
        self.pf_control_target.pf_update(inputs_pf_target, dt=self.dt)
        self.cpf_control_target.cpf_update(dt=self.dt)
        self.mpf_control_follower.mpf_update(inputs_mpf_follower, dt=self.dt)
        self.pf_control_follower.pf_update(inputs_pf_follower, dt=self.dt)
        self.cpf_control_follower.cpf_update(dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine_target.set_initial_conditions(init_cond["x0"], init_cond["y0"], init_cond["theta_m0"])
        self.pf_control_target.set_initial_conditions(init_cond["s0"])

        #pir = self.mpf_control_follower.point_in_reference(init_cond["x1"], init_cond["y1"], init_cond["x0"], init_cond["y0"], init_cond["theta_m0"])
        ry = self.mpf_control_follower.reference_yaw(init_cond["theta_m1"], init_cond["theta_m0"])
        self.mpf_control_follower.set_initial_conditions(init_cond["x1"], init_cond["y1"], ry)
        self.pf_control_follower.set_initial_conditions(init_cond["s1"])

        gammas = {"gamma0": init_cond["s0"], "gamma1": init_cond["s1"]}
        self.cpf_control_target.set_initial_conditions(gammas)
        self.cpf_control_follower.set_initial_conditions(gammas)

    def past_values(self):
        return (self.past_outputs, self.kine_target.past_state, self.pf_control_target.past_state, self.cpf_control_target.past_state, self.mpf_control_follower.past_state, self.pf_control_follower.past_state, self.cpf_control_follower.past_state)