"""

Author: Francisco Branco
Created: 19/05/2021

"""


import numpy as np
from math import pi, sqrt
import utils
import sys



class CPFContinuousController:
    def __init__(self, num_auv=2, id=0, k_csi=1, params=None, A_matrix=None, state_history=False, dt=1):
        self.id = id
        self.state_history = state_history
        self.dt = dt

        if A_matrix.shape[0] != A_matrix.shape[1]:
            print("Adjacency matrix is not the correct size")
            sys.exit()
        self.A_matrix = A_matrix
        self.D_matrix = np.zeros(A_matrix.shape)

        self.inputs = {}
        self.state = {}
        
        for i in range(num_auv):
            self.D_matrix[i][i] = sum(A_matrix[0])
            self.inputs["gamma" + str(i)] = 0
            self.state["error" + str(i)] = 0

        self.LD = np.linalg.inv(self.D_matrix).dot(self.D_matrix - self.A_matrix)

        if self.state_history:
            self.past_state = {}
            for i in range(num_auv):
                self.past_state["error" + str(i)] = []

        self.params = {
            "k_csi": k_csi,
            "num_auv": num_auv
        }
        
        for i in range(self.params["num_auv"]):
            self.params["norm" + str(i)] = params["norm" + str(i)]
            self.params["speed_profile" + str(i)] = params["speed_profile" + str(i)]

    def cpf_output(self):
        vd = self.vd()
        vd_dot = self.vd_dot()
        
        return (vd[self.id], vd_dot[self.id])

    def inputs_outputs(self):
        vd, vd_dot = self.cpf_output()
        outputs = {"velocity": vd, "velocity_dot": vd_dot}
        
        return (self.inputs.copy(), outputs)

    def set_initial_conditions(self, gammas):
        for key in gammas.keys():
            self.inputs[key] = gammas[key]
    
    def vd(self):
        gamma = self.get_gamma()
        error_gain = self.params["k_csi"]
        LD_matrix = self.LD
        speed_profile = np.zeros((self.params["num_auv"],))
        norm = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            speed_profile[i] = self.params["speed_profile" + str(i)]
            norm[i] = self.params["norm" + str(i)]

        v_correction = (-1) * error_gain * np.tanh(LD_matrix.dot(gamma))
        
        
        final_velocity = speed_profile + np.multiply(v_correction, norm)
        #print(final_velocity)
        return final_velocity

    def vd_dot(self):
        error_gain = self.params["k_csi"]
        vd = self.vd()
        gamma = self.get_gamma()

        differentiable_matrix = np.zeros((self.params["num_auv"], self.params["num_auv"]))
        norm = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            differentiable_matrix[i][i] = 1 / np.square(np.cosh(self.LD.dot(gamma)[i]))
            norm = self.params["norm" + str(i)]
        
        vd_dot = (-1) * error_gain * np.multiply(differentiable_matrix.dot(self.LD.dot(np.true_divide(vd, norm))), norm)
        #print(vd_dot)
        return vd_dot

    def get_gamma(self):
        gamma = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            gamma[i] = self.inputs["gamma" + str(i)]
        return gamma









class CPFDiscreteControllerETC:
    def __init__(self, num_auv=2, id=0, saturate=0, params=None, k_csi=1, A_matrix=None, delay=0, etc_type="Time", state_history=False, dt=1):
        self.id = id
        self.state_history = state_history
        self.dt = dt
        self.delay = delay
        self.saturate = saturate
        
        if etc_type != "Time" and etc_type != "State":
            print("Error: wrong etc_type. Either Time or State")
            sys.exit()
        self.etc_type = etc_type

        if A_matrix.shape[0] != A_matrix.shape[1]:
            print("Adjacency matrix is not the correct size")
            sys.exit()
        self.A_matrix = A_matrix
        self.D_matrix = np.zeros(A_matrix.shape)

        self.inputs = {"gamma" + str(self.id): 0}
        self.state = {}
        
        for i in range(num_auv):
            self.D_matrix[i][i] = sum(A_matrix[0])
            self.state["gamma_hat" + str(i)] = 0

        self.L = self.D_matrix - self.A_matrix
        self.Ls = (1/2) * (self.L + self.L.T)
        self.LD = np.linalg.inv(self.D_matrix).dot(self.D_matrix - self.A_matrix)

        self.comm_vector = np.linalg.inv(self.D_matrix).dot(self.A_matrix)[self.id]
        

        eigenvalues, _ = np.linalg.eig(self.Ls)
        if len(eigenvalues) > 1:
            cut_eigenvalues = np.delete(eigenvalues, np.argmin(eigenvalues))

        self.params = {
            "k_csi": k_csi,
            "num_auv": num_auv,
        }
        
        for i in range(self.params["num_auv"]):
            self.params["speed_profile" + str(i)] = params["speed_profile" + str(i)]
            self.params["norm" + str(i)] = params["norm" + str(i)]
        
        self.params["lambda2"] = cut_eigenvalues.min()
        self.params["lambdaN"] = cut_eigenvalues.max()

        # h time function parameters
        self.params["c0"] = params["c0"]
        self.params["c1"] = params["c1"]
        self.params["c2"] = params["c2"]

        # state equation parameters
        self.params["l"] = params["l"]
        self.params["k"] = params["k"]
        self.params["epsilon"] = params["epsilon"]
        self.params["epsilon0"] = params["epsilon0"]
        self.params["theta"] = params["theta"]
        self.params["alpha"] = self.params["l"] / self.params["lambda2"]
        self.params["sigma"] = (1 - self.params["epsilon"]) * self.params["k"] - self.params["alpha"]
        self.params["sigma_i"] = (self.params["k"] * self.D_matrix[self.id][self.id] / self.params["epsilon"]) + 2 * self.params["alpha"] * self.params["lambdaN"]

        if self.state_history:
            self.past_state = {}
            for i in range(num_auv):
                self.past_state["gamma_hat" + str(i)] = []
            self.past_state["broadcasts"] = []

    def cpf_output(self):
        vd = self.vd()
        vd_dot = self.vd_dot()
        
        return (vd, vd_dot)

    def inputs_outputs(self):
        vd, vd_dot = self.cpf_output()
        outputs = {"velocity": vd, "velocity_dot": vd_dot}
        
        return (self.inputs.copy(), outputs)

    def set_initial_conditions(self, gammas):
        for key in gammas.keys():
            self.inputs[key] = gammas[key]
    
    def vd(self):
        gamma = self.get_gamma_est()
        self_gamma, gamma = utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma)
        
        error_gain = self.params["k_csi"]
        speed_profile = self.params["speed_profile" + str(self.id)]
        #print(utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma))
        v_correction = (-1) * error_gain * np.tanh((self_gamma - np.dot(self.comm_vector, gamma)) * self.params["norm" + str(self.id)])
        final_velocity = speed_profile + v_correction
        """
        if abs(final_velocity) >= 1.75:
            print(gamma)
            print(utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma))
            print(final_velocity)
        """
        
        return final_velocity

    def vd_dot(self):
        error_gain = self.params["k_csi"]
        vd = self.vd()
        gamma = self.get_gamma_est()
        self_gamma, gamma = utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma)
        gamma_dot = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            gamma_dot[i] = self.params["speed_profile" + str(i)] / self.params["norm" + str(i)]

        differentiable_term = 1 / np.square(np.cosh(self_gamma - np.dot(self.comm_vector, gamma)))
        
        vd_dot = (-1) * error_gain * differentiable_term * (vd - np.dot(self.comm_vector, gamma_dot) * self.params["norm" + str(self.id)])
        #print(vd_dot)
        return vd_dot

    def get_gamma_est(self):
        gamma = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            gamma[i] = self.state["gamma_hat" + str(i)]

        utils.gamma_vector_check(gamma)

        """
        gamma_min = self.state["gamma_hat0"]
        index_min = 0
        gamma_max = self.state["gamma_hat0"]
        
        for i in range(self.params["num_auv"]):
            gamma[i] = self.state["gamma_hat" + str(i)]
            if gamma_min > gamma[i]:
                gamma_min = gamma[i]
                index_min = i
            if gamma_max < gamma[i]:
                gamma_max = gamma[i]

        if gamma_min >= 0 and gamma_min <= 0.25 and gamma_max >= 0.75 and gamma_max <= 1:
            gamma[index_min] = gamma[index_min] + 1
        """

        #print(gamma)
        
        return gamma

    def get_error(self):
        error = utils.gamma_difference(self.state["gamma_hat" + str(self.id)], self.inputs["gamma" + str(self.id)])
        #print("id ", self.id, "gamma hat ", self.state["gamma_hat" + str(self.id)], " gamma ", self.inputs["gamma" + str(self.id)], " error ", error)
        return error

    def cpf_update(self, dt=None):
        if dt is None:
            dt = self.dt

        for i in range(self.params["num_auv"]):
            self.state["gamma_hat" + str(i)] = self.state["gamma_hat" + str(i)] + self.params["speed_profile" + str(i)] / self.params["norm" + str(i)] * dt
            if self.state["gamma_hat" + str(i)] >= 1:
                self.state["gamma_hat" + str(i)] = self.state["gamma_hat" + str(i)] - 1

        if self.state_history:
            for state in self.state.keys():
                self.past_state[state].append(self.state[state])

    def check_update(self, t):
        if (self.time_trigger(t) > 0 and self.etc_type == "Time") or (self.state_trigger() > 0 and self.etc_type == "State"):
            if self.state_history:
                self.past_state["broadcasts"].append(t)
            # Updates its own state
            self.state["gamma_hat" + str(self.id)] = self.inputs["gamma" + str(self.id)]
            # Prepare to send states to other vehicles
            state = {}
            for i in range(self.params["num_auv"]):
                state["gamma_hat" + str(i)] = self.state["gamma_hat" + str(i)]
            return state
        else:
            return -1

    def reset(self, broadcast):
        for i in range(self.params["num_auv"]):
            if i != self.id:
                self.state["gamma_hat" + str(i)] = broadcast["gamma_hat" + str(i)]

    def time_trigger(self, t):
        error = self.get_error()
        h = self.params["c0"] + self.params["c1"] * np.exp(- self.params["c2"] * t)
        delta = np.abs(error) - h
        
        return delta

    def state_trigger(self):
        error = self.get_error()
        error_sum = 0
        for j in range(self.params["num_auv"]):
            error_sum = error_sum + self.A_matrix[self.id][j] * np.power(utils.gamma_difference(self.state["gamma_hat" + str(self.id)], self.state["gamma_hat" + str(j)]), 2)
        delta = np.power(error, 2) - (self.params["theta"] * (self.params["sigma"] / self.params["sigma_i"]) * error_sum + self.params["epsilon0"])
        
        return delta

    def simple_delay_output(self, t):
        if t - self.start_time < self.delay:
            
            self.start_time = t
        return


class CooperativeFormationControl(CPFDiscreteControllerETC):
    def __init__(self, num_auv=2, id=0, saturate=0, params=None, k_csi=1, A_matrix=None, delay=0, etc_type="Time", state_history=False, dt=1, virtual_centre=False, path=None, kf=1):
        super().__init__(num_auv=num_auv, id=id, saturate=saturate, params=params, k_csi=k_csi, A_matrix=A_matrix, delay=delay, etc_type=etc_type, state_history=state_history, dt=dt)
        self.virtual_centre = virtual_centre

        if virtual_centre:
            self.path = path
            self.kf = kf

            for i in range(self.params["num_auv"]):
                self.inputs["ef" + str(i)] = 0

            self.centre_gamma = 0.125
            self.past_centre_gamma = []

    def inputs_outputs(self):
        vd, vd_dot = self.cpf_output()
        outputs = {"velocity": vd, "velocity_dot": vd_dot}

        if self.virtual_centre:
            x, y = self.path.get_xy(self.inputs["gamma" + str(self.id)])
            theta = self.path.get_theta_c(self.inputs["gamma" + str(self.id)])
            outputs["centre_x"] = x
            outputs["centre_y"] = y
            outputs["centre_theta"] = theta
        
        return (self.inputs.copy(), outputs)

    def vd(self):
        gamma = self.get_gamma_est()
        self_gamma, gamma = utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma)
        
        error_gain = self.params["k_csi"]
        speed_profile = self.params["speed_profile" + str(self.id)]
        #print(utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma))
        v_correction = (-1) * error_gain * np.tanh((self_gamma - np.dot(self.comm_vector, gamma)) * self.params["norm" + str(self.id)])
        if self.virtual_centre:
            ef_sum = 0
            for i in range(self.params["num_auv"]):
                ef_sum = ef_sum + self.inputs["ef" + str(i)]
            ef = (1/self.params["num_auv"]) * ef_sum
            final_velocity = (1 - np.tanh(self.kf * ef)) * (speed_profile + v_correction)
        else:
            final_velocity = speed_profile + v_correction
        """
        if abs(final_velocity) >= 1.75:
            print(gamma)
            print(utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma))
            print(final_velocity)
        """
        
        return final_velocity

    def vd_dot(self):
        error_gain = self.params["k_csi"]
        vd = self.vd()
        gamma = self.get_gamma_est()
        self_gamma, gamma = utils.gamma_change_one(self.inputs["gamma" + str(self.id)], gamma)
        gamma_dot = np.zeros((self.params["num_auv"],))
        for i in range(self.params["num_auv"]):
            gamma_dot[i] = self.params["speed_profile" + str(i)] / self.params["norm" + str(i)]

        differentiable_term = 1 / np.square(np.cosh(self_gamma - np.dot(self.comm_vector, gamma)))

        speed_profile = self.params["speed_profile" + str(self.id)]
        v_correction = (-1) * error_gain * np.tanh((self_gamma - np.dot(self.comm_vector, gamma)) * self.params["norm" + str(self.id)])
        
        if self.virtual_centre:
            ef_sum = 0
            for i in range(self.params["num_auv"]):
                pass
                #ef_sum = ef_sum + np.sqrt(np.power(self.path.get_x(self.inputs["gamma" + str(i)]) - self.inputs["x" + str(i)], 2) + np.power(self.path.get_y(self.inputs["gamma" + str(i)]) - self.inputs["y" + str(i)], 2))
            ef = (1/self.params["num_auv"]) * ef_sum
            # NOT COMPLETE, REPLACE NUMBER 1
            vd_dot = (1) * (speed_profile + v_correction) + (1 - np.tanh(self.kf * ef)) * (-1) * error_gain * differentiable_term * (vd - np.dot(self.comm_vector, gamma_dot) * self.params["norm" + str(self.id)])
        else:
            vd_dot = (-1) * error_gain * differentiable_term * (vd - np.dot(self.comm_vector, gamma_dot) * self.params["norm" + str(self.id)])
        #print(vd_dot)
        return vd_dot

    def cfc_update(self, dt=None):
        if dt is None:
            dt = self.dt

        super().cpf_update(dt=dt)

        if self.virtual_centre:
            vd = self.vd()
            self.centre_gamma = self.centre_gamma + vd / self.path.total_distance * dt

            if self.state_history:
                self.past_centre_gamma.append(self.centre_gamma)