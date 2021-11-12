"""

Author: Francisco Branco
Created: 23/03/2021

"""


import numpy as np
from math import pi
import control as ct
import libct.kinematics as kn
import libct.pathfollowing as pf
import pathgeneration as pg
import libct.cooperativepathfollowing as cpf


def setup_simple_lapierre_kinematics_auv_system(some_path, gamma, k1, k2, k_delta, theta_a):
    kine = kn.Kinematics()
    name = "Lapierre"
    pf_control = pf.Lapierre(some_path, name, gamma, k1, k2, k_delta, theta_a)

    auv_system = ct.InterconnectedSystem(
        (kine, pf_control),
        name="AUVSYS",
        connections=(
            ("Lapierre.vehicle_x", "AUV.x_out"),
            ("Lapierre.vehicle_y", "AUV.y_out"),
            ("Lapierre.theta_m", "AUV.theta_m_out"),
            ("AUV.u", "Lapierre.u")),
        inplist=("AUV.velocity", "AUV.velocity_dot", "Lapierre.velocity", "Lapierre.velocity_dot"),
        inputs=("v", "v_dot", "v", "v_dot"),
        outlist=("AUV.x_out", "AUV.y_out", "AUV.theta_m_out"),
        outputs=("X", "Y", "theta_m"))
    
    return auv_system


def setup_simple_cooperative_lapierre_kinematics_auv_system(path1, path2, gamma, k1, k2, k_delta, theta_a, k_csi):

    A_matrix = np.array([[0, 1], [1, 0]])

    kine1 = kn.Kinematics(name="AUV1")
    pf_control1 = pf.Lapierre(some_path=path1, name="Lapierre1", gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a)

    kine2 = kn.Kinematics(name="AUV2")
    pf_control2 = pf.Lapierre(some_path=path2, name="Lapierre2", gamma=gamma, k1=k1, k2=k2, k_delta=k_delta, theta_a=theta_a)

    cpf_control = cpf.CPFContinuousController(name="CPF", k_csi=k_csi, num_auv=2, speed_profile=1, A_matrix=A_matrix)

    auv_system = ct.InterconnectedSystem(
        (kine1, pf_control1, kine2, pf_control2, cpf_control),
        name="AUVSYS",
        connections=(
            ("Lapierre1.vehicle_x", "AUV1.x_out"),
            ("Lapierre1.vehicle_y", "AUV1.y_out"),
            ("Lapierre1.theta_m", "AUV1.theta_m_out"),
            ("AUV1.u", "Lapierre1.u"),
            ("Lapierre2.vehicle_x", "AUV2.x_out"),
            ("Lapierre2.vehicle_y", "AUV2.y_out"),
            ("Lapierre2.theta_m", "AUV2.theta_m_out"),
            ("AUV2.u", "Lapierre2.u"),
            ("CPF.gamma_comm0", "Lapierre1.s"),
            ("CPF.gamma_comm1", "Lapierre2.s"),
            ("AUV1.velocity", "CPF.vd0"),
            ("AUV1.velocity_dot", "CPF.vd_dot0"),
            ("AUV2.velocity", "CPF.vd1"),
            ("AUV2.velocity_dot", "CPF.vd_dot1"),
            ("Lapierre1.velocity", "CPF.vd0"),
            ("Lapierre1.velocity_dot", "CPF.vd_dot0"),
            ("Lapierre2.velocity", "CPF.vd1"),
            ("Lapierre2.velocity_dot", "CPF.vd_dot1")
        ),
        outlist=("AUV1.x_out", "AUV1.y_out", "AUV1.theta_m_out", "AUV2.x_out", "AUV2.y_out", "AUV2.theta_m_out"),
        outputs=("X1", "Y1", "theta_m1", "X2", "Y2", "theta_m2")
    )

    return auv_system


