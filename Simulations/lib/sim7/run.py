"""

Author: Francisco Branco
Created: 17/02/2022
Description: Target Pursuit Simple example

"""



import numpy as np
from math import pi

import pathgeneration as pg
import lib.sim7.systembuild as sb
import lib.sim7.plot as plotting


def simulation():
    # Path parameters
    resolution = 40

    start_target = 0.0
    position_target = np.array([0.0, 0.0])
    orientation_target = -pi/2#0.0
    size_target = 15.0
    arc_target = 2*pi
    radius_target = size_target

    start_tracker = 0.0
    position_tracker = np.array([0.0, 0.0])
    orientation_tracker = -pi/2
    size_tracker = 5.0
    arc_tracker = 2*pi
    radius_tracker = size_tracker

    # Path creation
    p_target = pg.Path()
    circle_target = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target.append_path(circle_target)
    #p_target = pg.Path()
    #line_target = pg.Line(resolution, position_target, orientation_target, size_target, start_target)
    #p_target.append_path(line_target)

    p_tracker = pg.Path()
    circle_tracker = pg.Circle(resolution, position_tracker, orientation_tracker, arc_tracker, radius_tracker, start_tracker)
    p_tracker.append_path(circle_tracker)


    # Time parameters
    total_time = 350
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

    # EKF parameters
    F_matrix = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    
    Q_matrix = 10 * np.exp(-6) * np.array([[10, 0, 0, 0],
                              [0, 10, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

    R_matrix = 0.01

    ekf_params = {
        "F_matrix": F_matrix,
        "Q_matrix": Q_matrix,
        "R_matrix": R_matrix
    }

    # Amount of time in seconds the target is not moving at the beginning
    time_halted = 25

    target_speed = 0.2
    tracker_speed = 0.75

    # System creation along with initial conditions
    auv_system = sb.ASVMPFOnAUVTargetPursuit(
        p_target,
        p_tracker,
        target_speed=target_speed,
        tracker_speed=tracker_speed,
        pf_params=pf_params,
        ekf_params=ekf_params,
        time_halted=time_halted,
        history=True,
        dt=dt
    )

    ic = {
        "x_target": -4.0,
        "y_target": -10.0,
        "theta_m_target": theta_m,
        "s_target": s,
        "x_follower": 0.0,
        "y_follower": -12.0,
        "theta_m_follower": theta_m,
        "s_follower": s
    }
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)
        #input()

    # Plotting
    paths = {"p_target": p_target, "p_tracker": p_tracker}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)