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


class DoubleASVMPFETCOnTripleAUV:
    def __init__(self, path_target0, path_target1, path_target2, path_follower0, path_follower1, pf_params=None, cpf_params_target=None, cpf_params_follower=None, etc_type="Time", history=False, dt=1):
        self.dt = dt
        self.path_target0 = path_target0
        self.path_target1 = path_target1
        self.path_target2 = path_target2
        self.path_follower0 = path_follower0
        self.path_follower1 = path_follower1
        self.pf_prarams = pf_params
        self.cpf_params_target = cpf_params_target
        self.cpf_params_follower = cpf_params_follower
        
        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_outputs = {
                "x_target0": [],
                "y_target0": [],
                "theta_m_target0": [],
                "s_target0": [],
                "u_target0": [],
                "x_target1": [],
                "y_target1": [],
                "theta_m_target1": [],
                "s_target1": [],
                "u_target1": [],
                "x_target2": [],
                "y_target2": [],
                "theta_m_target2": [],
                "s_target2": [],
                "u_target2": [],
                "x_follower0": [],
                "y_follower0": [],
                "theta_m_follower0": [],
                "s_follower0": [],
                "u_follower0": [],
                "x_follower1": [],
                "y_follower1": [],
                "theta_m_follower1": [],
                "s_follower1": [],
                "u_follower1": [],
                "velocity_target0": [],
                "velocity_target1": [],
                "velocity_target2": [],
                "velocity_follower0": [],
                "velocity_follower1": []
            }
        else:
            state_history = False

        A_matrix_target = np.array([[0, 1, 1], [1, 0, 1], [1, 1, 0]])
        A_matrix_follower = np.array([[0, 1], [1, 0]])

        self.kine_target0 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target0 = pf.Lapierre(some_path=path_target0, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_target0 = cpf.CPFDiscreteControllerETC(num_auv=3, id=0, params=cpf_params_target, k_csi=cpf_params_target["k_csi0"], A_matrix=A_matrix_target, etc_type=etc_type, state_history=state_history, dt=dt)

        self.kine_target1 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target1 = pf.Lapierre(some_path=path_target1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_target1 = cpf.CPFDiscreteControllerETC(num_auv=3, id=1, params=cpf_params_target, k_csi=cpf_params_target["k_csi1"], A_matrix=A_matrix_target, etc_type=etc_type, state_history=state_history, dt=dt)

        self.kine_target2 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target2 = pf.Lapierre(some_path=path_target2, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_target2 = cpf.CPFDiscreteControllerETC(num_auv=3, id=2, params=cpf_params_target, k_csi=cpf_params_target["k_csi2"], A_matrix=A_matrix_target, etc_type=etc_type, state_history=state_history, dt=dt)

        self.mpf_control_follower0 = mpf.MovingPathFollowing(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_follower0 = pf.Lapierre(some_path=path_follower0, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_follower0 = cpf.CPFDiscreteControllerETC(num_auv=2, id=0, params=cpf_params_follower, k_csi=cpf_params_follower["k_csi0"], A_matrix=A_matrix_follower, etc_type=etc_type, state_history=state_history, dt=dt)
        
        self.mpf_control_follower1 = mpf.MovingPathFollowing(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_follower1 = pf.Lapierre(some_path=path_follower1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_follower1 = cpf.CPFDiscreteControllerETC(num_auv=2, id=1, params=cpf_params_follower, k_csi=cpf_params_follower["k_csi1"], A_matrix=A_matrix_follower, etc_type=etc_type, state_history=state_history, dt=dt)

    def update(self, t):
        # Get dictionaries setup
        inputs_kine_target0, outputs_kine_target0 = self.kine_target0.inputs_outputs()
        inputs_pf_target0, outputs_pf_target0 = self.pf_control_target0.inputs_outputs()
        inputs_cpf_target0, outputs_cpf_target0 = self.cpf_control_target0.inputs_outputs()

        inputs_kine_target1, outputs_kine_target1 = self.kine_target1.inputs_outputs()
        inputs_pf_target1, outputs_pf_target1 = self.pf_control_target1.inputs_outputs()
        inputs_cpf_target1, outputs_cpf_target1 = self.cpf_control_target1.inputs_outputs()

        inputs_kine_target2, outputs_kine_target2 = self.kine_target2.inputs_outputs()
        inputs_pf_target2, outputs_pf_target2 = self.pf_control_target2.inputs_outputs()
        inputs_cpf_target2, outputs_cpf_target2 = self.cpf_control_target2.inputs_outputs()

        inputs_mpf_follower0, outputs_mpf_follower0 = self.mpf_control_follower0.inputs_outputs()
        inputs_pf_follower0, outputs_pf_follower0 = self.pf_control_follower0.inputs_outputs()
        inputs_cpf_follower0, outputs_cpf_follower0 = self.cpf_control_follower0.inputs_outputs()

        inputs_mpf_follower1, outputs_mpf_follower1 = self.mpf_control_follower1.inputs_outputs()
        inputs_pf_follower1, outputs_pf_follower1 = self.pf_control_follower1.inputs_outputs()
        inputs_cpf_follower1, outputs_cpf_follower1 = self.cpf_control_follower1.inputs_outputs()


        self.cpf_control_target0.inputs["gamma0"] = outputs_pf_target0["s"]
        self.cpf_control_target1.inputs["gamma1"] = outputs_pf_target1["s"]
        self.cpf_control_target2.inputs["gamma2"] = outputs_pf_target2["s"]

        self.cpf_control_follower0.inputs["gamma0"] = outputs_pf_follower0["s"]
        self.cpf_control_follower1.inputs["gamma1"] = outputs_pf_follower1["s"]



        
        # Check if comms needs update
        broadcast_target0 = self.cpf_control_target0.check_update(t)
        if broadcast_target0 != -1:
            self.cpf_control_target1.reset(broadcast_target0)
            self.cpf_control_target2.reset(broadcast_target0)

        broadcast_target1 = self.cpf_control_target1.check_update(t)
        if broadcast_target1 != -1:
            self.cpf_control_target0.reset(broadcast_target1)
            self.cpf_control_target2.reset(broadcast_target1)

        broadcast_target2 = self.cpf_control_target2.check_update(t)
        if broadcast_target2 != -1:
            self.cpf_control_target0.reset(broadcast_target2)
            self.cpf_control_target1.reset(broadcast_target2)


        broadcast_follower0 = self.cpf_control_follower0.check_update(t)
        if broadcast_follower0 != -1:
            self.cpf_control_follower1.reset(broadcast_follower0)

        broadcast_follower1 = self.cpf_control_follower1.check_update(t)
        if broadcast_follower1 != -1:
            self.cpf_control_follower0.reset(broadcast_follower1)
        



        _, outputs_cpf_target0 = self.cpf_control_target0.inputs_outputs()
        _, outputs_cpf_target1 = self.cpf_control_target1.inputs_outputs()
        _, outputs_cpf_target2 = self.cpf_control_target2.inputs_outputs()

        _, outputs_cpf_follower0 = self.cpf_control_follower0.inputs_outputs()
        _, outputs_cpf_follower1 = self.cpf_control_follower1.inputs_outputs()



        # Connection of variables
        # AUV Target 0
        inputs_kine_target0["u"] = outputs_pf_target0["u"]
        inputs_kine_target0["velocity"] = outputs_cpf_target0["velocity"]
        inputs_kine_target0["velocity_dot"] = outputs_cpf_target0["velocity_dot"]
        
        inputs_pf_target0["x"] = outputs_kine_target0["x"]
        inputs_pf_target0["y"] = outputs_kine_target0["y"]
        inputs_pf_target0["theta_m"] = outputs_kine_target0["theta_m"]
        inputs_pf_target0["velocity"] = outputs_cpf_target0["velocity"]
        inputs_pf_target0["velocity_dot"] = outputs_cpf_target0["velocity_dot"]

        self.cpf_control_target0.inputs["gamma0"] = outputs_pf_target0["s"]

        # AUV Target 1
        inputs_kine_target1["u"] = outputs_pf_target1["u"]
        inputs_kine_target1["velocity"] = outputs_cpf_target1["velocity"]
        inputs_kine_target1["velocity_dot"] = outputs_cpf_target1["velocity_dot"]
        
        inputs_pf_target1["x"] = outputs_kine_target1["x"]
        inputs_pf_target1["y"] = outputs_kine_target1["y"]
        inputs_pf_target1["theta_m"] = outputs_kine_target1["theta_m"]
        inputs_pf_target1["velocity"] = outputs_cpf_target1["velocity"]
        inputs_pf_target1["velocity_dot"] = outputs_cpf_target1["velocity_dot"]

        self.cpf_control_target1.inputs["gamma1"] = outputs_pf_target1["s"]

        # AUV Target 2
        inputs_kine_target2["u"] = outputs_pf_target2["u"]
        inputs_kine_target2["velocity"] = outputs_cpf_target2["velocity"]
        inputs_kine_target2["velocity_dot"] = outputs_cpf_target2["velocity_dot"]
        
        inputs_pf_target2["x"] = outputs_kine_target2["x"]
        inputs_pf_target2["y"] = outputs_kine_target2["y"]
        inputs_pf_target2["theta_m"] = outputs_kine_target2["theta_m"]
        inputs_pf_target2["velocity"] = outputs_cpf_target2["velocity"]
        inputs_pf_target2["velocity_dot"] = outputs_cpf_target2["velocity_dot"]

        self.cpf_control_target2.inputs["gamma2"] = outputs_pf_target2["s"]

        # First ASV
        inputs_mpf_follower0["target_x"] = self.path_target1.get_x(outputs_pf_target1["s"])
        inputs_mpf_follower0["target_y"] = self.path_target1.get_y(outputs_pf_target1["s"])
        inputs_mpf_follower0["target_yaw"] = self.path_target1.get_theta_c(outputs_pf_target1["s"])
        inputs_mpf_follower0["target_u"] = 2 * pi / (self.path_target1.total_distance / self.cpf_params_target["speed_profile1"])
        inputs_mpf_follower0["target_velocity"] = self.cpf_params_target["speed_profile1"]
        inputs_mpf_follower0["follower_u"] = outputs_pf_follower0["u"] #pi * np.tanh(outputs_pf_follower0["u"])
        inputs_mpf_follower0["follower_velocity"] = outputs_cpf_follower0["velocity"]
        
        inputs_pf_follower0["x"] = outputs_mpf_follower0["x_ref"]
        inputs_pf_follower0["y"] = outputs_mpf_follower0["y_ref"]
        inputs_pf_follower0["theta_m"] = outputs_mpf_follower0["theta_m_ref"]
        inputs_pf_follower0["velocity"] = outputs_cpf_follower0["velocity"]
        inputs_pf_follower0["velocity_dot"] = outputs_cpf_follower0["velocity_dot"]

        self.cpf_control_follower0.inputs["gamma0"] = outputs_pf_follower0["s"]

        # Second ASV
        inputs_mpf_follower1["target_x"] = self.path_target1.get_x(outputs_pf_target1["s"])
        inputs_mpf_follower1["target_y"] = self.path_target1.get_y(outputs_pf_target1["s"])
        inputs_mpf_follower1["target_yaw"] = self.path_target1.get_theta_c(outputs_pf_target1["s"])
        inputs_mpf_follower1["target_u"] = 2 * pi / (self.path_target1.total_distance / self.cpf_params_target["speed_profile1"])
        inputs_mpf_follower1["target_velocity"] = self.cpf_params_target["speed_profile1"]
        inputs_mpf_follower1["follower_u"] = outputs_pf_follower1["u"] #pi * np.tanh(outputs_pf_follower1["u"])
        inputs_mpf_follower1["follower_velocity"] = outputs_cpf_follower1["velocity"]
        
        inputs_pf_follower1["x"] = outputs_mpf_follower1["x_ref"]
        inputs_pf_follower1["y"] = outputs_mpf_follower1["y_ref"]
        inputs_pf_follower1["theta_m"] = outputs_mpf_follower1["theta_m_ref"]
        inputs_pf_follower1["velocity"] = outputs_cpf_follower1["velocity"]
        inputs_pf_follower1["velocity_dot"] = outputs_cpf_follower1["velocity_dot"]

        self.cpf_control_follower1.inputs["gamma1"] = outputs_pf_follower1["s"]


        #print("u = " + str(outputs_pf_target1["u"]))
        #print("calc =" + str(2 * pi / (self.path_target1.total_distance / self.cpf_params_target["speed_profile1"])))


        outputs = {}
        outputs["x_target0"] = outputs_kine_target0["x"]
        outputs["y_target0"] = outputs_kine_target0["y"]
        outputs["theta_m_target0"] = outputs_kine_target0["theta_m"]
        outputs["s_target0"] = outputs_pf_target0["s"]
        outputs["u_target0"] = outputs_pf_target0["u"]
        outputs["x_target1"] = outputs_kine_target1["x"]
        outputs["y_target1"] = outputs_kine_target1["y"]
        outputs["theta_m_target1"] = outputs_kine_target1["theta_m"]
        outputs["s_target1"] = outputs_pf_target1["s"]
        outputs["u_target1"] = outputs_pf_target1["u"]
        outputs["x_target2"] = outputs_kine_target2["x"]
        outputs["y_target2"] = outputs_kine_target2["y"]
        outputs["theta_m_target2"] = outputs_kine_target2["theta_m"]
        outputs["s_target2"] = outputs_pf_target2["s"]
        outputs["u_target2"] = outputs_pf_target2["u"]
        outputs["x_follower0"] = outputs_mpf_follower0["x"]
        outputs["y_follower0"] = outputs_mpf_follower0["y"]
        outputs["theta_m_follower0"] = outputs_mpf_follower0["theta_m"]
        outputs["s_follower0"] = outputs_pf_follower0["s"]
        outputs["u_follower0"] = outputs_pf_follower0["u"]
        outputs["x_follower1"] = outputs_mpf_follower1["x"]
        outputs["y_follower1"] = outputs_mpf_follower1["y"]
        outputs["theta_m_follower1"] = outputs_mpf_follower1["theta_m"]
        outputs["s_follower1"] = outputs_pf_follower1["s"]
        outputs["u_follower1"] = outputs_pf_follower1["u"]
        outputs["velocity_target0"] = outputs_cpf_target0["velocity"]
        outputs["velocity_target1"] = outputs_cpf_target1["velocity"]
        outputs["velocity_target2"] = outputs_cpf_target2["velocity"]
        outputs["velocity_follower0"] = outputs_cpf_follower0["velocity"]
        outputs["velocity_follower1"] = outputs_cpf_follower1["velocity"]

        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine_target0.auv_update(inputs_kine_target0, dt=self.dt)
        self.pf_control_target0.pf_update(inputs_pf_target0, dt=self.dt)
        self.cpf_control_target0.cpf_update(dt=self.dt)

        self.kine_target1.auv_update(inputs_kine_target1, dt=self.dt)
        self.pf_control_target1.pf_update(inputs_pf_target1, dt=self.dt)
        self.cpf_control_target1.cpf_update(dt=self.dt)

        self.kine_target2.auv_update(inputs_kine_target2, dt=self.dt)
        self.pf_control_target2.pf_update(inputs_pf_target2, dt=self.dt)
        self.cpf_control_target2.cpf_update(dt=self.dt)

        self.mpf_control_follower0.mpf_update(inputs_mpf_follower0, dt=self.dt)
        self.pf_control_follower0.pf_update(inputs_pf_follower0, dt=self.dt)
        self.cpf_control_follower0.cpf_update(dt=self.dt)

        self.mpf_control_follower1.mpf_update(inputs_mpf_follower1, dt=self.dt)
        self.pf_control_follower1.pf_update(inputs_pf_follower1, dt=self.dt)
        self.cpf_control_follower1.cpf_update(dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        gammas_target = {"gamma0": init_cond["s_target0"], "gamma1": init_cond["s_target1"], "gamma2": init_cond["s_target2"]}

        self.kine_target0.set_initial_conditions(init_cond["x_target0"], init_cond["y_target0"], init_cond["theta_m_target0"])
        self.pf_control_target0.set_initial_conditions(init_cond["s_target0"])
        self.cpf_control_target0.set_initial_conditions(gammas_target)

        self.kine_target1.set_initial_conditions(init_cond["x_target1"], init_cond["y_target1"], init_cond["theta_m_target1"])
        self.pf_control_target1.set_initial_conditions(init_cond["s_target1"])
        self.cpf_control_target1.set_initial_conditions(gammas_target)

        self.kine_target2.set_initial_conditions(init_cond["x_target2"], init_cond["y_target2"], init_cond["theta_m_target2"])
        self.pf_control_target2.set_initial_conditions(init_cond["s_target2"])
        self.cpf_control_target2.set_initial_conditions(gammas_target)

        ry = self.mpf_control_follower0.reference_yaw(init_cond["theta_m_follower0"], init_cond["theta_m_target1"])
        self.mpf_control_follower0.set_initial_conditions(init_cond["x_follower0"], init_cond["y_follower0"], ry)
        self.pf_control_follower0.set_initial_conditions(init_cond["s_follower0"])

        ry = self.mpf_control_follower1.reference_yaw(init_cond["theta_m_follower1"], init_cond["theta_m_target1"])
        self.mpf_control_follower1.set_initial_conditions(init_cond["x_follower1"], init_cond["y_follower1"], ry)
        self.pf_control_follower1.set_initial_conditions(init_cond["s_follower1"])

        gammas_follower = {"gamma0": init_cond["s_follower0"], "gamma1": init_cond["s_follower1"]}
        self.cpf_control_follower0.set_initial_conditions(gammas_follower)
        self.cpf_control_follower1.set_initial_conditions(gammas_follower)

    def past_values(self):
        return (
            self.past_outputs,
            self.kine_target0.past_state,
            self.pf_control_target0.past_state,
            self.cpf_control_target0.past_state,
            self.kine_target1.past_state,
            self.pf_control_target1.past_state,
            self.cpf_control_target1.past_state,
            self.kine_target2.past_state,
            self.pf_control_target2.past_state,
            self.cpf_control_target2.past_state,
            self.mpf_control_follower0.past_state,
            self.pf_control_follower0.past_state,
            self.cpf_control_follower0.past_state,
            self.mpf_control_follower1.past_state,
            self.pf_control_follower1.past_state,
            self.cpf_control_follower1.past_state
        )