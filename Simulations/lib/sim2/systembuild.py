"""

Author: Francisco Branco
Created: 10/05/2020

"""


import numpy as np
from math import pi
import pathgeneration as pg
import lib.kinematics as kn
import lib.pathfollowing as pf
import lib.cooperativepathfollowing as cpf


class DoubleAUVCPFContinuousCommunications:
    def __init__(self, path1, path2, k_csi=1, cpf_params=None, gamma=1, k1=1, k2=1, k_delta=1, theta_a=1/pi, history=False, dt=1):
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
                "x1": [],
                "y1": [],
                "theta_m1": [],
                "velocity1": [],
                "s1": [],
                "u1": [],
                "velocity_dot1": []
                }
        else:
            state_history = False

        A_matrix = np.array([[0, 1],[1, 0]])

        self.kine0 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control0 = pf.Lapierre(some_path=path1, gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a, state_history=state_history, dt=dt)
        self.cpf_control0 = cpf.CPFContinuousController(num_auv=2, id=0, k_csi=k_csi, params=cpf_params, A_matrix=A_matrix, state_history=True, dt=dt)

        self.kine1 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control1 = pf.Lapierre(some_path=path2, gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a, state_history=state_history, dt=dt)
        self.cpf_control1 = cpf.CPFContinuousController(num_auv=2, id=1, k_csi=k_csi, params=cpf_params, A_matrix=A_matrix, state_history=True, dt=dt)

    def update(self, t):
        # Get dictionaries setup
        inputs_kine0, outputs_kine0 = self.kine0.inputs_outputs()
        inputs_pf0, outputs_pf0 = self.pf_control0.inputs_outputs()
        inputs_cpf0, outputs_cpf0 = self.cpf_control0.inputs_outputs()

        inputs_kine1, outputs_kine1 = self.kine1.inputs_outputs()
        inputs_pf1, outputs_pf1 = self.pf_control1.inputs_outputs()
        inputs_cpf1, outputs_cpf1 = self.cpf_control1.inputs_outputs()

        self.cpf_control0.inputs["gamma0"] = outputs_pf0["s"]
        self.cpf_control0.inputs["gamma1"] = outputs_pf1["s"]

        self.cpf_control1.inputs["gamma0"] = outputs_pf0["s"]
        self.cpf_control1.inputs["gamma1"] = outputs_pf1["s"]

        inputs_cpf0, outputs_cpf0 = self.cpf_control0.inputs_outputs()
        inputs_cpf1, outputs_cpf1 = self.cpf_control1.inputs_outputs()

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

        

        # Second AUV
        inputs_kine1["u"] = outputs_pf1["u"]
        inputs_kine1["velocity"] = outputs_cpf1["velocity"]
        inputs_kine1["velocity_dot"] = outputs_cpf1["velocity_dot"]
        
        inputs_pf1["x"] = outputs_kine1["x"]
        inputs_pf1["y"] = outputs_kine1["y"]
        inputs_pf1["theta_m"] = outputs_kine1["theta_m"]
        inputs_pf1["velocity"] = outputs_cpf1["velocity"]
        inputs_pf1["velocity_dot"] = outputs_cpf1["velocity_dot"]

        


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

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)



        # Update the system
        self.kine0.auv_update(inputs_kine0, dt=self.dt)
        self.pf_control0.pf_update(inputs_pf0, dt=self.dt)
        self.kine1.auv_update(inputs_kine1, dt=self.dt)
        self.pf_control1.pf_update(inputs_pf1, dt=self.dt)

        """
        # Fetch values to register them and maybe plot
        _, outputs_kine0 = self.kine0.inputs_outputs()
        _, outputs_pf0 = self.pf_control0.inputs_outputs()
        _, outputs_kine1 = self.kine1.inputs_outputs()
        _, outputs_pf1 = self.pf_control1.inputs_outputs()
        
        self.cpf_control0.inputs["gamma0"] = outputs_pf0["s"]
        self.cpf_control0.inputs["gamma1"] = outputs_pf1["s"]
        _, outputs_cpf0 = self.cpf_control0.inputs_outputs()
        self.cpf_control1.inputs["gamma0"] = outputs_pf0["s"]
        self.cpf_control1.inputs["gamma1"] = outputs_pf1["s"]
        _, outputs_cpf1 = self.cpf_control1.inputs_outputs()
        """
        
        
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
        return (self.past_outputs, self.kine0.past_state, self.pf_control0.past_state, self.kine1.past_state, self.pf_control1.past_state)


