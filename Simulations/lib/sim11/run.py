"""

Author: Francisco Branco
Created: 04/05/2022
Description: Cooperative Formation Control Example

"""



import numpy as np
import pickle
from math import pi

import pathgeneration as pg
import lib.sim11.systembuild as sb
import lib.sim11.plot as plotting


def simulation(file_name):
    if file_name != "":
        f = open("lib\sim11\\" + file_name + ".txt", 'wb')

    # Path parameters
    resolution = 40

    start_target = 0.0
    position_target = np.array([0.0, 0.0])
    orientation_target = -pi/2#0.0
    size_target = 50.0
    arc_target = 2*pi
    radius_target = size_target

    start_tracker = 0.0
    position_tracker = np.array([0.0, 0.0])
    orientation_tracker = -pi/2
    size_tracker = 14.0
    arc_tracker = 2*pi
    radius_tracker = size_tracker

    # Path creation
    p_target0 = pg.Path()
    circle_target0 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target - 10.0, start_target)
    p_target0.append_path(circle_target0)

    p_target1 = pg.Path()
    circle_target1 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target1.append_path(circle_target1)

    p_target2 = pg.Path()
    circle_target2 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target + 10.0, start_target)
    p_target2.append_path(circle_target2)


    p_tracker0 = pg.Path()
    circle_tracker0 = pg.Circle(resolution, position_tracker, 0, arc_tracker, radius_tracker, start_tracker)
    p_tracker0.append_path(circle_tracker0)

    p_tracker1 = pg.Path()
    circle_tracker1 = pg.Circle(resolution, position_tracker, orientation_tracker, arc_tracker, radius_tracker, start_tracker)
    p_tracker1.append_path(circle_tracker1)


    # Time parameters
    total_time = 1000
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10.0
    y = -10.0
    theta_m = 0.0
    s = 0.0

    target_speed = 0.2
    tracker_speed = target_speed * 2.5

    pf_params = {
        "gamma": 1.0,
        "k1": 1.0,
        "k2": 0.3,
        "k_delta": 0.5,
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
        "k_csi0": p_target0.total_distance / (p_target1.total_distance / (target_speed - 0.05)),
        "k_csi1": target_speed - 0.05,
        "k_csi2": p_target2.total_distance / (p_target1.total_distance / (target_speed - 0.05)),
        "norm0": p_target0.total_distance,
        "norm1": p_target1.total_distance,
        "norm2": p_target2.total_distance,
        "speed_profile0": p_target0.total_distance / (p_target1.total_distance / (target_speed)),
        "speed_profile1": target_speed,
        "speed_profile2": p_target2.total_distance / (p_target1.total_distance / (target_speed)), # 0.24
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
        "k_csi0": tracker_speed - 0.1, # tracker_speed
        "k_csi1": tracker_speed - 0.1, # tracker_speed
        "norm0": p_tracker0.total_distance,
        "norm1": p_tracker1.total_distance,
        "speed_profile0": tracker_speed,
        "speed_profile1": tracker_speed
    }

    cfc_params_formation = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": target_speed - 0.1,
        "k_csi1": target_speed - 0.1, # target_speed + 0.2
        "norm0": p_target1.total_distance,
        "norm1": p_target1.total_distance,
        "speed_profile0": target_speed,
        "speed_profile1": target_speed,
        "kf": 1,
        "kg": 1
    }

    smart_cpf = 0.01
    tracker = 0.1

    # Amount of time in seconds the target is not moving at the beginning
    time_halted = 0

    # System creation along with initial conditions
    auv_system = sb.DoubleASVCFCTripleAUV(
        p_target0,
        p_target1,
        p_target2,
        p_tracker0,
        p_tracker1,
        pf_params=pf_params,
        cpf_params_target=cpf_params_target,
        cpf_params_tracker=cpf_params_tracker,
        cfc_params_formation=cfc_params_formation,
        time_halted=time_halted,
        etc_type="Time",
        smart_cpf=smart_cpf,
        tracker=tracker,
        history=True,
        dt=dt
    )

    ic = {
        "x_target0": -40.0,
        "y_target0": -40.0,
        "theta_m_target0": theta_m,
        "s_target0": 0.125,
        "x_target1": -50.0,
        "y_target1": -50.0,
        "theta_m_target1": theta_m,
        "s_target1": 0.125,
        "x_target2": -60.0,
        "y_target2": -60.0,
        "theta_m_target2": theta_m,
        "s_target2": 0.125,
        "x_follower0": -99.0,
        "y_follower0": -99.0,
        "theta_m_follower0": theta_m,
        "s_follower0": s,
        "x_follower1": -100.0,
        "y_follower1": -100.0,
        "theta_m_follower1": theta_m,
        "s_follower1": s
    }
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)
        #input()


    # Plotting
    paths = {"p_target0": p_target0, "p_target1": p_target1, "p_target2": p_target2, "p_tracker0": p_tracker0, "p_tracker1": p_tracker1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    if file_name != "":
        pickle.dump(paths, f)
        pickle.dump(num_points, f)
        pickle.dump(total_time, f)
        pickle.dump(resolution, f)
        pickle.dump(T, f)
        pickle.dump(past_values, f)
        f.close()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)