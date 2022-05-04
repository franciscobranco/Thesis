"""

Author: Francisco Branco
Created: 04/11/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim5.systembuild as sb
import lib.sim5.plot as plotting


def simulation():
    # Path parameters
    resolution = 40

    start_target = 0
    position_target = np.array([0, 0])
    orientation_target = -pi/2
    size_target = 15.0
    arc_target = 2*pi
    radius_target = size_target

    start = 0
    position0 = np.array([0, 0])
    position1 = np.array([0, 0])
    orientation = -pi/2
    size = 5.0
    arc = 2*pi
    radius = size

    # Path creation
    p_target = pg.Path()
    circle_target = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target.append_path(circle_target)

    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position0, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position1, 0, arc, radius, start)
    p1.append_path(circle1)

    # Time parameters
    total_time = 350
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10
    y = -10
    theta_m = 0
    s = 0

    target_speed = 0.2

    pf_params = {
        "gamma": 1,
        "k1": 1,
        "k2": 0.3,
        "k_delta": 1,
        "theta_a": 0.8
    }

    cpf_params = {
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
        "norm0": p0.total_distance,
        "norm1": p1.total_distance,
        "speed_profile0": 0.5,
        "speed_profile1": 0.5
    }


    # System creation along with initial conditions
    auv_system = sb.DoubleAUVMPFETC(p_target, p0, p1, target_speed=target_speed, pf_params=pf_params, cpf_params=cpf_params, etc_type="Time", history=True, dt=dt)
    ic = {"x_target": x, "y_target": y, "theta_m_target": theta_m, "s_target": s, "x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y - 5, "theta_m1": theta_m, "s1": s}
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)

    # Plotting
    paths = {"p_target": p_target, "p0": p0, "p1": p1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)