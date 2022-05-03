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
import lib.movingpathfollowing as mpf
import lib.extendedkalmanfilter as ekf
import lib.complementaryfilter as cf




class DoubleASVMPFOnAUVTargetPursuitCF:
    def __init__(self,
                 path_target1,
                 path_tracker0,
                 path_tracker1,
                 pf_params=None,
                 cpf_params_target=None,
                 cpf_params_tracker=None,
                 ekf_params=None,
                 ckf_params=None,
                 time_halted=50,
                 etc_type="Time",
                 history=False, dt=1):

        self.dt = dt
        self.path_target1 = path_target1
        self.path_tracker0 = path_tracker0
        self.path_tracker1 = path_tracker1
        self.pf_prarams = pf_params
        self.cpf_params_target = cpf_params_target
        self.cpf_params_tracker = cpf_params_tracker
        self.ekf_params = ekf_params
        self.ckf_params = ckf_params
        self.time_halted = time_halted

        self.last_cf_ekf_time = 0

        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_outputs = {
                "x_target1": [],
                "y_target1": [],
                "theta_m_target1": [],
                "s_target1": [],
                "u_target1": [],
                "x_pred_target1": [],
                "y_pred_target1": [],
                "x_tracker0": [],
                "y_tracker0": [],
                "theta_m_tracker0": [],
                "s_tracker0": [],
                "u_tracker0": [],
                "x_tracker1": [],
                "y_tracker1": [],
                "theta_m_tracker1": [],
                "s_tracker1": [],
                "u_tracker1": [],
                "velocity_target1": [],
                "velocity_tracker0": [],
                "velocity_tracker1": [],
                "x_ekf": [],
                "y_ekf": [],
                "theta_ekf": [],
                "velocity_ekf": [],
                "theta_dot_ekf": [],
                "range0": [],
                "range1": []
            }
        else:
            state_history = False


        self.A_matrix_target = np.array([[0, 1, 1], [1, 0, 1], [1, 1, 0]])
        self.A_matrix_tracker = np.array([[0, 1], [1, 0]])


        self.kine_target1 = kn.Kinematics(saturate=0, state_history=state_history, dt=dt)
        self.pf_control_target1 = pf.Lapierre(some_path=path_target1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.ckf_target1 = cf.ComplementaryKalmanFilter(A_matrix=ckf_params["A_matrix"], B_matrix=ckf_params["B_matrix"], C_matrix=ckf_params["C_matrix"], Q_matrix=ckf_params["Q_matrix"], Dopvar=ckf_params["doppler_var"], state_history=state_history, dt=dt)
        self.dms_target1 = cf.DopplerMeasureSimulation(variance=ckf_params["doppler_var"], sampling_period=0)

        
        self.mpf_control_tracker0 = mpf.MovingPathFollowingTest(target_velocity="Multiple", saturate=0, state_history=state_history, dt=dt)
        self.pf_control_tracker0 = pf.Lapierre(some_path=path_tracker0, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_tracker0 = cpf.CPFDiscreteControllerETC(num_auv=2, id=0, params=cpf_params_tracker, k_csi=cpf_params_tracker["k_csi0"], A_matrix=self.A_matrix_tracker, etc_type=etc_type, state_history=state_history, dt=dt)
        self.rms_tracker0 = ekf.RangeMeasureSimulation(R=ekf_params["R_matrix"][0][0], sampling_period=2)
        self.ekf_tracker = ekf.ExtendedKalmanFilter(F_matrix=ekf_params["F_matrix"], Q_matrix=ekf_params["Q_matrix"], R_matrix=ekf_params["R_matrix"], state_history=state_history, dt=dt)
        
        self.mpf_control_tracker1 = mpf.MovingPathFollowingTest(target_velocity="Multiple", saturate=0, state_history=state_history, dt=dt)
        self.pf_control_tracker1 = pf.Lapierre(some_path=path_tracker1, gamma=pf_params["gamma"], k1=pf_params["k1"], k2=pf_params["k2"], k_delta=pf_params["k_delta"], theta_a=pf_params["theta_a"], state_history=state_history, dt=dt)
        self.cpf_control_tracker1 = cpf.CPFDiscreteControllerETC(num_auv=2, id=1, params=cpf_params_tracker, k_csi=cpf_params_tracker["k_csi1"], A_matrix=self.A_matrix_tracker, etc_type=etc_type, state_history=state_history, dt=dt)
        self.rms_tracker1 = ekf.RangeMeasureSimulation(R=ekf_params["R_matrix"][1][1], sampling_period=2)

    def update(self, t):
        # Get dictionaries setup
        inputs_kine_target1, outputs_kine_target1 = self.kine_target1.inputs_outputs()
        inputs_pf_target1, outputs_pf_target1 = self.pf_control_target1.inputs_outputs()
        inputs_ckf_target1, outputs_ckf_target1 = self.ckf_target1.inputs_outputs()


        inputs_mpf_tracker0, outputs_mpf_tracker0 = self.mpf_control_tracker0.inputs_outputs()
        inputs_pf_tracker0, outputs_pf_tracker0 = self.pf_control_tracker0.inputs_outputs()
        inputs_cpf_tracker0, outputs_cpf_tracker0 = self.cpf_control_tracker0.inputs_outputs()
        inputs_ekf_tracker, outputs_ekf_tracker = self.ekf_tracker.inputs_outputs()

        inputs_mpf_tracker1, outputs_mpf_tracker1 = self.mpf_control_tracker1.inputs_outputs()
        inputs_pf_tracker1, outputs_pf_tracker1 = self.pf_control_tracker1.inputs_outputs()
        inputs_cpf_tracker1, outputs_cpf_tracker1 = self.cpf_control_tracker1.inputs_outputs()

        self.cpf_control_tracker0.inputs["gamma0"] = outputs_pf_tracker0["s"]
        self.cpf_control_tracker1.inputs["gamma1"] = outputs_pf_tracker1["s"]


        # EKF Prediction and Update
        inputs_ekf_tracker["tracker_x0"] = outputs_mpf_tracker0["x"]
        inputs_ekf_tracker["tracker_y0"] = outputs_mpf_tracker0["y"]
        inputs_ekf_tracker["tracker_x1"] = outputs_mpf_tracker1["x"]
        inputs_ekf_tracker["tracker_y1"] = outputs_mpf_tracker1["y"]
        tracker0_pos = np.array([outputs_mpf_tracker0["x"], outputs_mpf_tracker0["y"]])
        tracker1_pos = np.array([outputs_mpf_tracker1["x"], outputs_mpf_tracker1["y"]])
        target_pos = np.array([outputs_kine_target1["x"], outputs_kine_target1["y"]])
        y_k0 = self.rms_tracker0.measurement(t, tracker0_pos, target_pos)
        y_k1 = self.rms_tracker1.measurement(t, tracker1_pos, target_pos)
        if t < 150 or t > 200:
            inputs_ekf_tracker["range0"] = y_k0
            inputs_ekf_tracker["range1"] = y_k1
        else:
            inputs_ekf_tracker["range0"] = None
            inputs_ekf_tracker["range1"] = None

        self.ekf_tracker.ekf_update(inputs_ekf_tracker, t)
        inputs_ekf_tracker, outputs_ekf_tracker = self.ekf_tracker.inputs_outputs()


        #CKF Prediction and Update
        vel_m = self.dms_target1.measurement(t, outputs_kine_target1["theta_m"], self.cpf_params_target["speed_profile1"])
        inputs_ckf_target1["vx_dop"] = vel_m[0]
        inputs_ckf_target1["vy_dop"] = vel_m[1]
        if t - self.last_cf_ekf_time > 5:
            self.last_cf_ekf_time = t
            inputs_ckf_target1["x_EKF"] = outputs_ekf_tracker["x"]
            inputs_ckf_target1["y_EKF"] = outputs_ekf_tracker["y"]
            inputs_ckf_target1["R"] = np.array([[self.ekf_tracker.P_aposteriori[0][0], self.ekf_tracker.P_aposteriori[0][1]],
                                                [self.ekf_tracker.P_aposteriori[1][0], self.ekf_tracker.P_aposteriori[1][1]]])
        else:
            inputs_ckf_target1["x_EKF"] = None
            inputs_ckf_target1["y_EKF"] = None
            inputs_ckf_target1["R"] = None
        
        self.ckf_target1.ckf_update(inputs_ckf_target1, t)
        inputs_ckf_target1, outputs_ckf_target1 = self.ckf_target1.inputs_outputs()


        # CPF ETC update
        broadcast_tracker0 = self.cpf_control_tracker0.check_update(t)
        if broadcast_tracker0 != -1:
            self.cpf_control_tracker1.reset(broadcast_tracker0)

        broadcast_tracker1 = self.cpf_control_tracker1.check_update(t)
        if broadcast_tracker1 != -1:
            self.cpf_control_tracker0.reset(broadcast_tracker1)

        _, outputs_cpf_tracker0 = self.cpf_control_tracker0.inputs_outputs()
        _, outputs_cpf_tracker1 = self.cpf_control_tracker1.inputs_outputs()


        # Connection of variables
        # AUV Target
        inputs_kine_target1["u"] = outputs_pf_target1["u"]
        
        if t < self.time_halted:
            inputs_kine_target1["velocity"] = 0.0
            inputs_pf_target1["velocity"] = 0.0
        else:
            inputs_kine_target1["velocity"] = self.cpf_params_target["speed_profile1"]
            inputs_pf_target1["velocity"] = self.cpf_params_target["speed_profile1"]

        inputs_kine_target1["velocity_dot"] = 0

        inputs_pf_target1["x"] = outputs_ckf_target1["x"]
        inputs_pf_target1["y"] = outputs_ckf_target1["y"]
        inputs_pf_target1["theta_m"] = outputs_kine_target1["theta_m"]
        
        inputs_pf_target1["velocity_dot"] = 0


        # ASV
        if t < self.time_halted:
            inputs_mpf_tracker0["target_x"] = -4.
            inputs_mpf_tracker0["target_y"] = -10.
            inputs_mpf_tracker0["target_x_dot"] = 0.
            inputs_mpf_tracker0["target_y_dot"] = 0.
            inputs_mpf_tracker1["target_x"] = -4.
            inputs_mpf_tracker1["target_y"] = -10.
            inputs_mpf_tracker1["target_x_dot"] = 0.
            inputs_mpf_tracker1["target_y_dot"] = 0.
        else:
            inputs_mpf_tracker0["target_x"] = outputs_ekf_tracker["x"]
            inputs_mpf_tracker0["target_y"] = outputs_ekf_tracker["y"]
            inputs_mpf_tracker0["target_x_dot"] = outputs_ekf_tracker["x_dot"]
            inputs_mpf_tracker0["target_y_dot"] = outputs_ekf_tracker["y_dot"]
            inputs_mpf_tracker1["target_x"] = outputs_ekf_tracker["x"]
            inputs_mpf_tracker1["target_y"] = outputs_ekf_tracker["y"]
            inputs_mpf_tracker1["target_x_dot"] = outputs_ekf_tracker["x_dot"]
            inputs_mpf_tracker1["target_y_dot"] = outputs_ekf_tracker["y_dot"]

        inputs_mpf_tracker0["target_yaw"] = 0
        inputs_mpf_tracker0["target_u"] = 0
        inputs_mpf_tracker1["target_yaw"] = 0
        inputs_mpf_tracker1["target_u"] = 0
        
        inputs_mpf_tracker0["follower_u"] = outputs_pf_tracker0["u"]
        inputs_mpf_tracker0["follower_velocity"] = outputs_cpf_tracker0["velocity"]
        inputs_mpf_tracker1["follower_u"] = outputs_pf_tracker1["u"]
        inputs_mpf_tracker1["follower_velocity"] = outputs_cpf_tracker1["velocity"]
        
        inputs_pf_tracker0["x"] = outputs_mpf_tracker0["x_ref"]
        inputs_pf_tracker0["y"] = outputs_mpf_tracker0["y_ref"]
        inputs_pf_tracker0["theta_m"] = outputs_mpf_tracker0["theta_m_ref"]
        inputs_pf_tracker0["velocity"] = outputs_cpf_tracker0["velocity"]
        inputs_pf_tracker0["velocity_dot"] = outputs_cpf_tracker0["velocity_dot"]
        inputs_pf_tracker1["x"] = outputs_mpf_tracker1["x_ref"]
        inputs_pf_tracker1["y"] = outputs_mpf_tracker1["y_ref"]
        inputs_pf_tracker1["theta_m"] = outputs_mpf_tracker1["theta_m_ref"]
        inputs_pf_tracker1["velocity"] = outputs_cpf_tracker1["velocity"]
        inputs_pf_tracker1["velocity_dot"] = outputs_cpf_tracker0["velocity_dot"]

        self.cpf_control_tracker0.inputs["gamma0"] = outputs_pf_tracker0["s"]
        self.cpf_control_tracker1.inputs["gamma1"] = outputs_pf_tracker1["s"]



        outputs = {}
        outputs["x_target1"] = outputs_kine_target1["x"]
        outputs["y_target1"] = outputs_kine_target1["y"]
        outputs["theta_m_target1"] = outputs_kine_target1["theta_m"]
        outputs["s_target1"] = outputs_pf_target1["s"]
        outputs["u_target1"] = outputs_pf_target1["u"]
        outputs["x_pred_target1"] = outputs_ckf_target1["x"]
        outputs["y_pred_target1"] = outputs_ckf_target1["y"]
        outputs["x_tracker0"] = outputs_mpf_tracker0["x"]
        outputs["y_tracker0"] = outputs_mpf_tracker0["y"]
        outputs["theta_m_tracker0"] = outputs_mpf_tracker0["theta_m"]
        outputs["s_tracker0"] = outputs_pf_tracker0["s"]
        outputs["u_tracker0"] = outputs_pf_tracker0["u"]
        outputs["x_tracker1"] = outputs_mpf_tracker1["x"]
        outputs["y_tracker1"] = outputs_mpf_tracker1["y"]
        outputs["theta_m_tracker1"] = outputs_mpf_tracker1["theta_m"]
        outputs["s_tracker1"] = outputs_pf_tracker1["s"]
        outputs["u_tracker1"] = outputs_pf_tracker1["u"]
        if t < self.time_halted:
            outputs["velocity_target1"] = 0
        else:
            outputs["velocity_target1"] = self.cpf_params_target["speed_profile1"]
        outputs["velocity_tracker0"] = outputs_cpf_tracker0["velocity"]
        outputs["velocity_tracker1"] = outputs_cpf_tracker1["velocity"]
        outputs["x_ekf"] = outputs_ekf_tracker["x"]
        outputs["y_ekf"] = outputs_ekf_tracker["y"]
        outputs["theta_ekf"] = outputs_ekf_tracker["theta"]
        outputs["velocity_ekf"] = np.sqrt(np.power(outputs_ekf_tracker["x_dot"], 2) + np.power(outputs_ekf_tracker["y_dot"], 2))
        outputs["theta_dot_ekf"] = outputs_ekf_tracker["theta_dot"]
        outputs["range0"] = y_k0
        outputs["range1"] = y_k1


        # Save outputs for plotting
        if self.history:
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)


        # Update the system
        self.kine_target1.auv_update(inputs_kine_target1, dt=self.dt)
        self.pf_control_target1.pf_update(inputs_pf_target1, dt=self.dt)


        self.mpf_control_tracker0.mpf_update(inputs_mpf_tracker0, dt=self.dt)
        self.pf_control_tracker0.pf_update(inputs_pf_tracker0, dt=self.dt)
        self.cpf_control_tracker0.cpf_update(dt=self.dt)

        self.mpf_control_tracker1.mpf_update(inputs_mpf_tracker1, dt=self.dt)
        self.pf_control_tracker1.pf_update(inputs_pf_tracker1, dt=self.dt)
        self.cpf_control_tracker1.cpf_update(dt=self.dt)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine_target1.set_initial_conditions(init_cond["x_target1"], init_cond["y_target1"], init_cond["theta_m_target1"])
        self.pf_control_target1.set_initial_conditions(init_cond["s_target1"])
        ic_ckf1 = self.ckf_target1.state.copy()
        ic_ckf1["x"] = init_cond["x_target1"]
        ic_ckf1["y"] = init_cond["y_target1"]
        ic_ckf1["vc_x"] = 0
        ic_ckf1["vc_y"] = 0
        self.ckf_target1.set_initial_conditions(ic_ckf1)



        ry0 = self.mpf_control_tracker0.reference_yaw(init_cond["theta_m_follower0"], init_cond["theta_m_target1"])
        self.mpf_control_tracker0.set_initial_conditions(init_cond["x_follower0"], init_cond["y_follower0"], ry0)
        self.pf_control_tracker0.set_initial_conditions(init_cond["s_follower0"])

        ry1 = self.mpf_control_tracker1.reference_yaw(init_cond["theta_m_follower1"], init_cond["theta_m_target1"])
        self.mpf_control_tracker1.set_initial_conditions(init_cond["x_follower1"], init_cond["y_follower1"], ry1)
        self.pf_control_tracker1.set_initial_conditions(init_cond["s_follower1"])

        gammas_tracker = {"gamma0": init_cond["s_follower0"], "gamma1": init_cond["s_follower1"]}
        self.cpf_control_tracker0.set_initial_conditions(gammas_tracker)
        self.cpf_control_tracker1.set_initial_conditions(gammas_tracker)

        ic_ekf = {
            "x": init_cond["x_target1"],
            "y": init_cond["y_target1"],
            "x_dot": 0,
            "y_dot": 0
        }
        self.ekf_tracker.set_initial_conditions(ic_ekf)

    def past_values(self):
        return (
            self.past_outputs,
            self.kine_target1.past_state,
            self.pf_control_target1.past_state,
            self.ckf_target1.past_state,
            self.mpf_control_tracker0.past_state,
            self.pf_control_tracker0.past_state,
            self.cpf_control_tracker0.past_state,
            self.mpf_control_tracker1.past_state,
            self.pf_control_tracker1.past_state,
            self.cpf_control_tracker1.past_state,
            self.ekf_tracker.past_state
        )