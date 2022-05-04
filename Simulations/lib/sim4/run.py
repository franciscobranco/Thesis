"""

Author: Francisco Branco
Created: 02/10/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim4.systembuild as sb
import lib.sim4.plot as plotting


def simulation():
    # Path parameters
    resolution = 40
    start = 0
    position0 = np.array([0, 0])
    position1 = np.array([0, 0])
    orientation = -pi/2
    size = 15.0
    arc = 2*pi
    radius = size

    # Path creation
    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position0, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position1, orientation, arc, radius - 10, start)
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
    factor = 10

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
        "k_csi0": 0.1,
        "k_csi1": 0.2,
        "norm0": p0.total_distance,
        "norm1": p1.total_distance * 10,
        "speed_profile0": 0.25,
        "speed_profile1": p1.total_distance * 10 / p0.total_distance * 0.25
    }


    # System creation along with initial conditions
    auv_system = sb.DoubleAUVMPFETCTest(p0, p1, pf_params=pf_params, cpf_params=cpf_params, etc_type="Time", factor=factor, history=True, dt=dt)
    ic = {"x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y - 5, "theta_m1": theta_m, "s1": s}
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)

    # Plotting
    paths = {"p0": p0, "p1": p1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values, factor)