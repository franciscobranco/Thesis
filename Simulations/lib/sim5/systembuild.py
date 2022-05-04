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
    def __init__(self, path_target, path0, path1, target_speed=1, pf_params=None, cpf_params=None, etc_type="Time", history=False, dt=1):
        self.dt = dt
        self.path_target = path_target
        self.path0 = path0
        self.path1 = path1
        self.pf_prarams = pf_params
        self.target_speed = target_speed
        self.cpf_params = cpf_params
        
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
        self.pf_control_target = pf.Lapierre(some_path=path_target, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)

        self.mpf_control_follower0 = mpf.MovingPathFollowingTest(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_follower0 = pf.Lapierre(some_path=path0, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_follower0 = cpf.CPFDiscreteControllerETC(num_auv=2, id=0, params=cpf_params, k_csi=cpf_params["k_csi0"], A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=dt)
        
        self.mpf_control_follower1 = mpf.MovingPathFollowingTest(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_follower1 = pf.Lapierre(some_path=path1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_follower1 = cpf.CPFDiscreteControllerETC(num_auv=2, id=1, params=cpf_params, k_csi=cpf_params["k_csi1"], A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=dt)

    def update(self, t):
        
        # Get dictionaries setup
        inputs_kine_target, outputs_kine_target = self.kine_target.inputs_outputs()
        inputs_pf_target, outputs_pf_target = self.pf_control_target.inputs_outputs()

        inputs_mpf_follower0, outputs_mpf_follower0 = self.mpf_control_follower0.inputs_outputs()
        inputs_pf_follower0, outputs_pf_follower0 = self.pf_control_follower0.inputs_outputs()
        inputs_cpf_follower0, outputs_cpf_follower0 = self.cpf_control_follower0.inputs_outputs()

        inputs_mpf_follower1, outputs_mpf_follower1 = self.mpf_control_follower1.inputs_outputs()
        inputs_pf_follower1, outputs_pf_follower1 = self.pf_control_follower1.inputs_outputs()
        inputs_cpf_follower1, outputs_cpf_follower1 = self.cpf_control_follower1.inputs_outputs()


        self.cpf_control_follower0.inputs["gamma0"] = outputs_pf_follower0["s"]
        self.cpf_control_follower1.inputs["gamma1"] = outputs_pf_follower1["s"]



        
        # Check if comms needs update
        broadcast0 = self.cpf_control_follower0.check_update(t)
        if broadcast0 != -1:
            self.cpf_control_follower1.reset(broadcast0)

        broadcast1 = self.cpf_control_follower1.check_update(t)
        if broadcast1 != -1:
            self.cpf_control_follower0.reset(broadcast1)
        





        _, outputs_cpf_follower0 = self.cpf_control_follower0.inputs_outputs()
        _, outputs_cpf_follower1 = self.cpf_control_follower1.inputs_outputs()



        # Connection of variables
        # Virtual Target AUV
        inputs_kine_target["u"] = outputs_pf_target["u"]
        inputs_kine_target["velocity"] = self.target_speed
        inputs_kine_target["velocity_dot"] = 0
        
        inputs_pf_target["x"] = outputs_kine_target["x"]
        inputs_pf_target["y"] = outputs_kine_target["y"]
        inputs_pf_target["theta_m"] = outputs_kine_target["theta_m"]
        inputs_pf_target["velocity"] = self.target_speed
        inputs_pf_target["velocity_dot"] = 0

        # First ASV
        inputs_mpf_follower0["target_x"] = outputs_kine_target["x"]
        inputs_mpf_follower0["target_y"] = outputs_kine_target["y"]
        inputs_mpf_follower0["target_yaw"] = outputs_kine_target["theta_m"]
        inputs_mpf_follower0["target_u"] = outputs_pf_target["u"]
        inputs_mpf_follower0["target_velocity"] = self.target_speed
        inputs_mpf_follower0["follower_u"] = outputs_pf_follower0["u"]
        inputs_mpf_follower0["follower_velocity"] = outputs_cpf_follower0["velocity"]
        
        inputs_pf_follower0["x"] = outputs_mpf_follower0["x_ref"]
        inputs_pf_follower0["y"] = outputs_mpf_follower0["y_ref"]
        inputs_pf_follower0["theta_m"] = outputs_mpf_follower0["theta_m_ref"]
        inputs_pf_follower0["velocity"] = outputs_cpf_follower0["velocity"]
        inputs_pf_follower0["velocity_dot"] = outputs_cpf_follower0["velocity_dot"]

        self.cpf_control_follower0.inputs["gamma0"] = outputs_pf_follower0["s"]

        # Second ASV
        inputs_mpf_follower1["target_x"] = outputs_kine_target["x"]
        inputs_mpf_follower1["target_y"] = outputs_kine_target["y"]
        inputs_mpf_follower1["target_yaw"] = outputs_kine_target["theta_m"]
        inputs_mpf_follower1["target_u"] = outputs_pf_target["u"]
        inputs_mpf_follower1["target_velocity"] = self.target_speed
        inputs_mpf_follower1["follower_u"] = outputs_pf_follower1["u"]
        inputs_mpf_follower1["follower_velocity"] = outputs_cpf_follower1["velocity"]
        
        inputs_pf_follower1["x"] = outputs_mpf_follower1["x_ref"]
        inputs_pf_follower1["y"] = outputs_mpf_follower1["y_ref"]
        inputs_pf_follower1["theta_m"] = outputs_mpf_follower1["theta_m_ref"]
        inputs_pf_follower1["velocity"] = outputs_cpf_follower1["velocity"]
        inputs_pf_follower1["velocity_dot"] = outputs_cpf_follower1["velocity_dot"]

        self.cpf_control_follower1.inputs["gamma1"] = outputs_pf_follower1["s"]


        outputs = {}
        outputs["x_target"] = outputs_kine_target["x"]
        outputs["y_target"] = outputs_kine_target["y"]
        outputs["theta_m_target"] = outputs_kine_target["theta_m"]
        outputs["s_target"] = outputs_pf_target["s"]
        outputs["u_target"] = outputs_pf_target["u"]
        outputs["x0"] = outputs_mpf_follower0["x"]
        outputs["y0"] = outputs_mpf_follower0["y"]
        outputs["theta_m0"] = outputs_mpf_follower0["theta_m"]
        outputs["s0"] = outputs_pf_follower0["s"]
        outputs["u0"] = outputs_pf_follower0["u"]
        outputs["x1"] = outputs_mpf_follower1["x"]
        outputs["y1"] = outputs_mpf_follower1["y"]
        outputs["theta_m1"] = outputs_mpf_follower1["theta_m"]
        outputs["s1"] = outputs_pf_follower1["s"]
        outputs["u1"] = outputs_pf_follower1["u"]
        outputs["velocity0"] = outputs_cpf_follower0["velocity"]
        outputs["velocity1"] = outputs_cpf_follower1["velocity"]

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine_target.auv_update(inputs_kine_target, dt=self.dt)
        self.pf_control_target.pf_update(inputs_pf_target, dt=self.dt)
        self.mpf_control_follower0.mpf_update(inputs_mpf_follower0, dt=self.dt)
        self.pf_control_follower0.pf_update(inputs_pf_follower0, dt=self.dt)
        self.mpf_control_follower1.mpf_update(inputs_mpf_follower1, dt=self.dt)
        self.pf_control_follower1.pf_update(inputs_pf_follower1, dt=self.dt)
        self.cpf_control_follower0.cpf_update(dt=self.dt)
        self.cpf_control_follower1.cpf_update(dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine_target.set_initial_conditions(init_cond["x_target"], init_cond["y_target"], init_cond["theta_m_target"])
        self.pf_control_target.set_initial_conditions(init_cond["s_target"])

        #pir = self.mpf_control_follower.point_in_reference(init_cond["x1"], init_cond["y1"], init_cond["x0"], init_cond["y0"], init_cond["theta_m0"])
        ry = self.mpf_control_follower0.reference_yaw(init_cond["theta_m0"], init_cond["theta_m_target"])
        self.mpf_control_follower0.set_initial_conditions(init_cond["x0"], init_cond["y0"], ry)
        self.pf_control_follower0.set_initial_conditions(init_cond["s0"])

        ry = self.mpf_control_follower1.reference_yaw(init_cond["theta_m1"], init_cond["theta_m_target"])
        self.mpf_control_follower1.set_initial_conditions(init_cond["x1"], init_cond["y1"], ry)
        self.pf_control_follower1.set_initial_conditions(init_cond["s1"])

        gammas = {"gamma0": init_cond["s0"], "gamma1": init_cond["s1"]}
        self.cpf_control_follower0.set_initial_conditions(gammas)
        self.cpf_control_follower1.set_initial_conditions(gammas)

    def past_values(self):
        return (self.past_outputs, self.kine_target.past_state, self.pf_control_target.past_state, self.mpf_control_follower0.past_state, self.pf_control_follower0.past_state, self.cpf_control_follower0.past_state, self.mpf_control_follower1.past_state, self.pf_control_follower1.past_state, self.cpf_control_follower1.past_state)