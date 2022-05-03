"""

Author: Francisco Branco
Created: 02/05/2022
Description: Two ASVs target pursuit with double range measurement example and complementary Kalman filter compensation

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim10.systembuild as sb
import lib.sim10.plot as plotting


def simulation():
    # Path parameters
    resolution = 40

    start_target = 0.0
    position_target = np.array([0.0, 0.0])
    orientation_target = -pi/2#0.0
    size_target = 20.0
    arc_target = 2*pi
    radius_target = size_target

    start_tracker = 0.0
    position_tracker = np.array([0.0, 0.0])
    orientation_tracker = -pi/2
    size_tracker = 7.0
    arc_tracker = 2*pi
    radius_tracker = size_tracker

    # Path creation
    p_target0 = pg.Path()
    circle_target0 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target - 5.0, start_target)
    p_target0.append_path(circle_target0)

    p_target1 = pg.Path()
    circle_target1 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target1.append_path(circle_target1)

    p_target2 = pg.Path()
    circle_target2 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target + 5.0, start_target)
    p_target2.append_path(circle_target2)


    p_tracker0 = pg.Path()
    circle_tracker0 = pg.Circle(resolution, position_tracker, 0, arc_tracker, radius_tracker, start_tracker)
    p_tracker0.append_path(circle_tracker0)

    p_tracker1 = pg.Path()
    circle_tracker1 = pg.Circle(resolution, position_tracker, orientation_tracker, arc_tracker, radius_tracker, start_tracker)
    p_tracker1.append_path(circle_tracker1)


    # Time parameters
    total_time = 600
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10.0
    y = -10.0
    theta_m = 0.0
    s = 0.0

    pf_params = {
        "gamma": 1.0,
        "k1": 1.0,
        "k2": 0.3,
        "k_delta": 1.0,
        "theta_a": 0.8
    }

    cpf_params_target = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": 0.25,
        "k_csi1": 0.25,
        "k_csi2": 0.25,
        "norm0": p_target0.total_distance,
        "norm1": p_target1.total_distance,
        "norm2": p_target2.total_distance,
        "speed_profile0": p_target0.total_distance / (p_target2.total_distance / 0.2),
        "speed_profile1": p_target1.total_distance / (p_target2.total_distance / 0.2),
        "speed_profile2": 0.2
    }

    cpf_params_tracker = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": 0.5,
        "k_csi1": 0.5,
        "norm0": p_tracker0.total_distance,
        "norm1": p_tracker1.total_distance,
        "speed_profile0": 0.7,
        "speed_profile1": 0.7
    }

    # EKF parameters
    F_matrix = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    
    Q_matrix = 10 * np.exp(-6) * np.array([[10, 0, 0, 0],
                                           [0, 10, 0, 0],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])

    R_matrix = np.array([[0.01, 0], [0, 0.01]])

    ekf_params = {
        "F_matrix": F_matrix,
        "Q_matrix": Q_matrix,
        "R_matrix": R_matrix
    }

    A_matrix = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    B_matrix = np.array([[dt, 0],
                         [0, dt],
                         [0, 0],
                         [0, 0]])
    C_matrix = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])

    doppler_var = 1#np.array([[0.01, 0], [0, 0.01]])

    ckf_params = {
        "A_matrix": A_matrix,
        "B_matrix": B_matrix,
        "C_matrix": C_matrix,
        "Q_matrix": Q_matrix,
        "doppler_var": doppler_var
    }

    # Amount of time in seconds the target is not moving at the beginning
    time_halted = 0


    # System creation along with initial conditions
    auv_system = sb.DoubleASVMPFOnAUVTargetPursuitCF(
        p_target1,
        p_tracker0,
        p_tracker1,
        pf_params=pf_params,
        cpf_params_target=cpf_params_target,
        cpf_params_tracker=cpf_params_tracker,
        ekf_params=ekf_params,
        ckf_params=ckf_params,
        time_halted=time_halted,
        etc_type="Time",
        history=True,
        dt=dt
    )

    ic = {
        "x_target1": -2.0,
        "y_target1": -10.0,
        "theta_m_target1": theta_m,
        "s_target1": s,
        "x_follower0": 0.0,
        "y_follower0": -12.0,
        "theta_m_follower0": theta_m,
        "s_follower0": s,
        "x_follower1": -1.0,
        "y_follower1": -14.0,
        "theta_m_follower1": theta_m,
        "s_follower1": s
    }
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)
        #input()




    # Plotting
    paths = {"p_target1": p_target1, "p_tracker0": p_tracker0, "p_tracker1": p_tracker1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)