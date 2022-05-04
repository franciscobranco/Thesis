"""

Author: Francisco Branco
Created: 10/09/2021
Description: ETC Cooperative Path Following example

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim3.systembuild as sb
import lib.sim3.plot as plotting


def simulation():
    # Path parameters
    resolution = 40
    start = 0
    position1 = np.array([0, 0])
    position2 = np.array([0, 0])
    orientation = -pi/2
    size = 15.0
    arc = 2*pi
    radius = size

    # Path creation
    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position1, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position2, orientation, arc, radius - 5, start)
    p1.append_path(circle1)

    # Time parameters
    total_time = 150
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = 0
    y = 0
    theta_m = 0
    s = 0


    nominal_speed_profile = 0.8

    # ETC Parameters
    cpf_params = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "norm0": p0.total_distance,
        "norm1": p1.total_distance,
        "speed_profile0": nominal_speed_profile,
        "speed_profile1": nominal_speed_profile * p1.total_distance / p0.total_distance
    }

    # System creation along with initial conditions
    auv_system = sb.DoubleAUVCPFETC(p0, p1, k_csi=0.1, gamma=1, k1=1, k2=0.3, k_delta=1, theta_a=0.8, cpf_params=cpf_params, etc_type="Time", history=True, dt=dt)
    ic = {"x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y, "theta_m1": theta_m, "s1": s}
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)

    # Plotting
    paths = {"p0": p0, "p1": p1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)