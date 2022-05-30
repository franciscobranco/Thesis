"""

Author: Francisco Branco
Created: 04/11/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import pickle
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim5.systembuild as sb
import lib.sim5.plot as plotting


def simulation(file_name):
    if file_name != "":
        f = open("lib\sim5\\" + file_name + ".txt", 'wb')

    # Path parameters
    resolution = 40

    start_target = 0
    position_target = np.array([0, 0])
    orientation_target = -pi/2
    size_target = 25.0
    arc_target = 2*pi
    radius_target = size_target

    start = 0
    position_tracker = np.array([0, 0])
    orientation = -pi/2
    size = 10.0
    arc = 2*pi
    radius = size

    # Path creation
    p_target = pg.Path()
    circle_target = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target.append_path(circle_target)

    p_tracker = pg.Path()
    circle_tracker = pg.Circle(resolution, position_tracker, orientation, arc, radius, start)
    p_tracker.append_path(circle_tracker)

    # Time parameters
    total_time = 500
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10
    y = -10
    theta_m = 0
    s = 0

    target_speed = 0.2
    tracker_speed = 0.7

    pf_params = {
        "gamma": 1,
        "k1": 1,
        "k2": 0.3,
        "k_delta": 1,
        "theta_a": 0.8
    }


    # System creation along with initial conditions
    auv_system = sb.DoubleAUVMPFETC(p_target, p_tracker, target_speed=target_speed, pf_params=pf_params, tracker_speed=tracker_speed, history=True, dt=dt)
    ic = {"x_target": x, "y_target": y, "theta_m_target": theta_m, "s_target": s, "x_tracker": x, "y_tracker": y, "theta_m_tracker": theta_m, "s_tracker": s}
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)

    # Plotting
    paths = {"p_target": p_target, "p_tracker": p_tracker}
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