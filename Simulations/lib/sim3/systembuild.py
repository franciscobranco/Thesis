"""

Author: Francisco Branco
Created: 10/05/2020

"""


import numpy as np
from math import pi

import lib.kinematics as kn
import lib.pathfollowing as pf
import lib.cooperativepathfollowing as cpf


class DoubleAUVCPFETC:
    def __init__(self, path0, path1, k_csi=1, gamma=1, k1=1, k2=1, k_delta=1, theta_a=1/pi, cpf_params=None, etc_type="Time", history=False, dt=1):
        self.dt = dt
        
        #self.inputs = {"velocity": 0, "velocity_dot": 0}
        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_outputs = {
                "x0": [],
                "y0": [],
                "theta_m0": [],
                "velocity0": [],
                "s0": [],
                "u0": [],
                "velocity_dot0": [],
                "gamma00": [],
                "gamma01": [],
                "x1": [],
                "y1": [],
                "theta_m1": [],
                "velocity1": [],
                "s1": [],
                "u1": [],
                "velocity_dot1": [],
                "gamma10": [],
                "gamma11": [],
            }
        else:
            state_history = False

        A_matrix = np.array([[0, 1],[1, 0]])

        self.kine0 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control0 = pf.Lapierre(some_path=path0, gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a, state_history=state_history, dt=dt)
        self.cpf_control0 = cpf.CPFDiscreteControllerETC(num_auv=2, id=0, params=cpf_params, k_csi=k_csi, A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=1)
        
        self.kine1 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control1 = pf.Lapierre(some_path=path1, gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a, state_history=state_history, dt=dt)
        self.cpf_control1 = cpf.CPFDiscreteControllerETC(num_auv=2, id=1, params=cpf_params, k_csi=k_csi, A_matrix=A_matrix, etc_type=etc_type, state_history=state_history, dt=1)
        
    def update(self, t):
        
        # Get dictionaries setup
        inputs_kine0, outputs_kine0 = self.kine0.inputs_outputs()
        inputs_pf0, outputs_pf0 = self.pf_control0.inputs_outputs()
        inputs_cpf0, outputs_cpf0 = self.cpf_control0.inputs_outputs()

        inputs_kine1, outputs_kine1 = self.kine1.inputs_outputs()
        inputs_pf1, outputs_pf1 = self.pf_control1.inputs_outputs()
        inputs_cpf1, outputs_cpf1 = self.cpf_control1.inputs_outputs()

        

        self.cpf_control0.inputs["gamma0"] = outputs_pf0["s"]
        self.cpf_control1.inputs["gamma1"] = outputs_pf1["s"]




        # Check if comms needs update
        broadcast0 = self.cpf_control0.check_update(t)
        if broadcast0 != -1:
            self.cpf_control1.reset(broadcast0)

        broadcast1 = self.cpf_control1.check_update(t)
        if broadcast1 != -1:
            self.cpf_control0.reset(broadcast1)


        _, outputs_cpf0 = self.cpf_control0.inputs_outputs()
        _, outputs_cpf1 = self.cpf_control1.inputs_outputs()



        # Connection of variables
        # First AUV
        inputs_kine0["u"] = outputs_pf0["u"]
        inputs_kine0["velocity"] = outputs_cpf0["velocity"]
        inputs_kine0["velocity_dot"] = outputs_cpf0["velocity_dot"]
        
        inputs_pf0["x"] = outputs_kine0["x"]
        inputs_pf0["y"] = outputs_kine0["y"]
        inputs_pf0["theta_m"] = outputs_kine0["theta_m"]
        inputs_pf0["velocity"] = outputs_cpf0["velocity"]
        inputs_pf0["velocity_dot"] = outputs_cpf0["velocity_dot"]

        self.cpf_control0.inputs["gamma0"] = outputs_pf0["s"]

        # Second AUV
        inputs_kine1["u"] = outputs_pf1["u"]
        inputs_kine1["velocity"] = outputs_cpf1["velocity"]
        inputs_kine1["velocity_dot"] = outputs_cpf1["velocity_dot"]
        
        inputs_pf1["x"] = outputs_kine1["x"]
        inputs_pf1["y"] = outputs_kine1["y"]
        inputs_pf1["theta_m"] = outputs_kine1["theta_m"]
        inputs_pf1["velocity"] = outputs_cpf1["velocity"]
        inputs_pf1["velocity_dot"] = outputs_cpf1["velocity_dot"]

        self.cpf_control1.inputs["gamma1"] = outputs_pf1["s"]


        outputs = {}
        outputs["x0"] = outputs_kine0["x"]
        outputs["y0"] = outputs_kine0["y"]
        outputs["theta_m0"] = outputs_kine0["theta_m"]
        outputs["x1"] = outputs_kine1["x"]
        outputs["y1"] = outputs_kine1["y"]
        outputs["theta_m1"] = outputs_kine1["theta_m"]
        outputs["s0"] = outputs_pf0["s"]
        outputs["u0"] = outputs_pf0["u"]
        outputs["s1"] = outputs_pf1["s"]
        outputs["u1"] = outputs_pf1["u"]
        outputs["velocity0"] = outputs_cpf0["velocity"]
        outputs["velocity_dot0"] = outputs_cpf0["velocity_dot"]
        outputs["velocity1"] = outputs_cpf1["velocity"]
        outputs["velocity_dot1"] = outputs_cpf1["velocity_dot"]
        outputs["gamma00"] = self.cpf_control0.state["gamma_hat0"]
        outputs["gamma01"] = self.cpf_control0.state["gamma_hat1"]
        outputs["gamma10"] = self.cpf_control1.state["gamma_hat0"]
        outputs["gamma11"] = self.cpf_control1.state["gamma_hat1"]

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine0.auv_update(inputs_kine0, dt=self.dt)
        self.pf_control0.pf_update(inputs_pf0, dt=self.dt)
        self.cpf_control0.cpf_update(dt=self.dt)
        self.kine1.auv_update(inputs_kine1, dt=self.dt)
        self.pf_control1.pf_update(inputs_pf1, dt=self.dt)
        self.cpf_control1.cpf_update(dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine0.set_initial_conditions(init_cond["x0"], init_cond["y0"], init_cond["theta_m0"])
        self.pf_control0.set_initial_conditions(init_cond["s0"])

        self.kine1.set_initial_conditions(init_cond["x1"], init_cond["y1"], init_cond["theta_m1"])
        self.pf_control1.set_initial_conditions(init_cond["s1"])

        gammas = {"gamma0": init_cond["s0"], "gamma1": init_cond["s1"]}
        self.cpf_control0.set_initial_conditions(gammas)
        self.cpf_control1.set_initial_conditions(gammas)

    def past_values(self):
        return (self.past_outputs, self.kine0.past_state, self.pf_control0.past_state, self.cpf_control0.past_state, self.kine1.past_state, self.pf_control1.past_state, self.cpf_control1.past_state)