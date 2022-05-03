"""

Author: Francisco Branco
Created: 10/05/2020

"""


from math import pi
import lib.kinematics as kn
import lib.pathfollowing as pf


class SimpleAUVPathFollowing:
    def __init__(self, some_path, gamma=1, k1=1, k2=1, k_delta=1, theta_a=1/pi, history=False, dt=1):
        self.dt = dt
        
        self.inputs = {"velocity": 0, "velocity_dot": 0}
        self.time = []
        
        self.history = history
        if history:
            state_history = True
            self.past_inputs = {
                "velocity": [],
                "velocity_dot": []
                }
            self.past_outputs = {
                "x": [],
                "y": [],
                "theta_m": [],
                "u": [],
                "s": [],
                "s1": [],
                "y1": []
                }
        else:
            state_history = False

        self.kine = kn.Kinematics(saturate=1.5, state_history=state_history, dt=dt)
        self.pf_control = pf.Lapierre(some_path=some_path, gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a, state_history=state_history, dt=dt)

    def update(self, t, inputs=None):
        if inputs is not None:
            self.inputs = inputs.copy()
        
        inputs_kine, outputs_kine = self.kine.inputs_outputs()
        inputs_pf, outputs_pf = self.pf_control.inputs_outputs()

        for key in inputs_kine.keys():
            if key in self.inputs.keys():
                inputs_kine[key] = self.inputs[key]
            if key in outputs_kine.keys():
                inputs_kine[key] = outputs_kine[key]
            if key in outputs_pf.keys():
                inputs_kine[key] = outputs_pf[key]
                
        for key in inputs_pf.keys():
            if key in self.inputs.keys():
                inputs_pf[key] = self.inputs[key]
            if key in outputs_kine.keys():
                inputs_pf[key] = outputs_kine[key]
            if key in outputs_pf.keys():
                inputs_pf[key] = outputs_pf[key]

        # Update the system
        self.kine.auv_update(inputs_kine, dt=self.dt)
        self.pf_control.pf_update(inputs_pf, dt=self.dt)

        _, outputs = self.kine.inputs_outputs()
        _, aux = self.pf_control.inputs_outputs()
        outputs.update(aux)

        if self.history:
            # Save outputs for plotting
            for key in self.past_inputs.keys():
                self.past_inputs[key].append(self.inputs[key])
            for key in self.past_outputs.keys():
                self.past_outputs[key].append(outputs[key])

        self.time.append(t)

        return outputs

    def set_initial_conditions(self, init_cond):
        self.kine.set_initial_conditions(init_cond["x"], init_cond["y"], init_cond["theta_m"])
        self.pf_control.set_initial_conditions(init_cond["s"])

    def past_values(self):
        return (self.past_outputs, self.past_inputs, self.kine.past_state, self.pf_control.past_state)


