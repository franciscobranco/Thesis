"""

Author: Francisco Branco
Created: 10/03/2021

"""

import numpy as np
from math import pi, sqrt
import control as ct
import utils



class CPFContinuousController(ct.NonlinearIOSystem):
    
    def __init__(self, name="CPF_ct", k_csi=1, num_auv=2, speed_profile=1, A_matrix=None):
        #get communication matrix
        #get functions for error
        #inputs and outputs for each vehicle
        #communication interface?
        self.A_matrix = A_matrix
        self.D_matrix = np.zeros(A_matrix.shape)
        input_list = []
        output_list = []
        state_list = []
        for i in range(num_auv):
            self.D_matrix[i][i] = sum(A_matrix[0])
            input_list.append("gamma_comm" + str(i))
            output_list.append("vd" + str(i))
            output_list.append("vd_dot" + str(i))
            state_list.append("error" + str(i))
            #state_list.append("gamma" + str(i))

        self.LD = np.linalg.inv(self.D_matrix).dot(self.D_matrix - self.A_matrix)

        params = {
            "k_csi": k_csi,
            "num_auv": num_auv,
            "speed_profile": speed_profile}

        super().__init__(
            updfcn=self.cpf_update,
            outfcn=self.cpf_output,
            name=name,
            params=params,
            inputs=input_list,
            outputs=output_list,
            states=state_list)
        
    def cpf_update(self, t, x, u, params={}):
        error = []
        for i in range(params.get("num_auv")):
            error.append(x[i])
        LD_matrix = self.LD
        speed_profile = speed_profile = np.full((params.get("num_auv"),), params.get("speed_profile"))
        error_gain = params.get("k_csi")
        error_update = LD_matrix.dot(speed_profile) - error_gain * LD_matrix.dot(np.tanh(error))

        #print("update_list = " + str(error_update))
        return error_update

    def cpf_output(self, t, x, u, params={}):
        vd = self.vd(x, u, params)
        vd_dot = self.vd_dot(x, u, params)
        gamma = self.get_gamma(x, u, params)

        output_list = []
        for i in range(params.get("num_auv")):
            output_list.append(vd[i])
            output_list.append(vd_dot[i])
        #print("output_list = " + str(output_list))
        return output_list

    def get_gamma(self, x, u, params):
        gamma = []
        for i in range(params.get("num_auv")):
            gamma.append(u[i])
        #print("gamma = " + str(gamma))
        return gamma
    
    def vd(self, x, u, params):
        gamma = self.get_gamma(x, u, params)
        error_gain = params.get("k_csi")
        LD_matrix = self.LD
        speed_profile = np.full((params.get("num_auv"),), params.get("speed_profile"))
        v_correction = (-1) * error_gain * np.tanh(LD_matrix.dot(gamma))
        final_velocity = speed_profile + v_correction

        #print("result = " + str(gamma))

        return final_velocity

    def vd_dot(self, x, u, params):
        error_gain = params.get("k_csi")
        vd = self.vd(x, u, params)
        gamma = self.get_gamma(x, u, params)

        differentiable_matrix = np.zeros((params.get("num_auv"), params.get("num_auv")))
        for i in range(params.get("num_auv")):
            differentiable_matrix[i][i] = 1 / np.square(np.cosh(self.LD.dot(gamma)[i]))

        vd_dot = (-1) * error_gain * differentiable_matrix.dot(self.LD.dot(vd))        

        return vd_dot



class CooperativeController(ct.NonlinearIOSystem):
    def __init__(self, name="CPF_ct", k_csi=1, comm_type=0, num_auv=1, auv_i=0, speed_profile=1, A_matrix=None):
        #get communication matrix
        #get functions for error
        #inputs and outputs for each vehicle
        #communication interface?
        self.A_matrix = A_matrix
        self.D_matrix = np.zeros(A_matrix.shape)
        input_list = []
        output_list = ["vd", "vd_dot"]
        state_list = []
        for i in range(num_auv):
            self.D_matrix[i][i] = sum(A_matrix[0])
            input_list.append("gamma_comm" + str(i))
            output_list.append("gamma_out" + str(i))
            state_list.append("error" + str(i))
            state_list.append("gamma" + str(i))
        input_list.append("s")

        self.LD = np.linalg.inv(self.D_matrix).dot(self.D_matrix - self.A_matrix)

        params = {
            "k_csi": k_csi,
            "comm_type": comm_type,
            "num_auv": num_auv,
            "auv_i": auv_i,
            "speed_profile": speed_profile}

        super().__init__(
            updfcn=self.cpf_update,
            outfcn=self.cpf_output,
            name=name,
            params=params,
            inputs=input_list,
            outputs=output_list,
            states=state_list)
        
    def cpf_update(self, t, x, u, params={}):
        error = []
        for i in range(params.get("num_auv")):
            error.append(x[i])
        LD_matrix = self.LD
        speed_profile = params.get("speed_profile")
        error_gain = params.get("k_csi")
        error_update = LD_matrix.dot(speed_profile) - error_gain * LD_matrix.dot(np.tanh(error))
        gamma_update = self.vd(x, u, params)

        update_list = []
        for element in error_update + gamma_update:
            update_list.append(element)
        #print("update_list = " + str(update_list))
        return update_list

    def cpf_output(self, t, x, u, params={}):
        vd = self.vd(x, u, params)
        vd_dot = self.vd_dot(x, u, params)
        i = params.get("auv_i")
        gamma = self.get_gamma(x, u, params)

        output_list = [vd[i], vd_dot[i]]
        for i in range(params.get("num_auv")):
            output_list.append(gamma[i])
        #print("output_list = " + str(output_list))
        return output_list

    def get_gamma(self, x, u, params):
        if params.get("comm_type") == 0:
            gamma = []
            for i in range(params.get("num_auv")):
                if i == params.get("auv_i"):
                    if u[-1] < 0:
                        gamma.append(0)
                    elif u[-1] > 1:
                        gamma.append(1)
                    else:
                        gamma.append(u[0])
                else:
                    if u[i + 1] < 0:
                        gamma.append(0)
                    elif u[i + 1] > 1:
                        gamma.append(1)
                    else:
                        gamma.append(u[i + 1])
            #print("gamma = " + str(gamma))
            return gamma
        else:
            pass
    
    def vd(self, x, u, params):
        gamma = self.get_gamma(x, u, params)
        error_gain = params.get("k_csi")
        LD_matrix = self.LD
        speed_profile = np.full((params.get("num_auv"),), params.get("speed_profile"))
        v_correction = (-1) * error_gain * np.tanh(LD_matrix.dot(gamma))
        final_velocity = speed_profile + v_correction

        print("result = " + str(gamma))

        return final_velocity

    def vd_dot(self, x, u, params):
        error_gain = params.get("k_csi")
        vd = self.vd(x, u, params)
        gamma = self.get_gamma(x, u, params)

        differentiable_matrix = np.zeros((params.get("num_auv"), params.get("num_auv")))
        for i in range(params.get("num_auv")):
            differentiable_matrix[i][i] = 1 / np.square(np.cosh(self.LD.dot(gamma)[i]))

        vd_dot = (-1) * error_gain * differentiable_matrix.dot(self.LD.dot(vd))        

        return vd_dot


