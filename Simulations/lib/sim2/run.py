"""

Author: Francisco Branco
Created: 24/08/2021
Description: Continuous Communication Cooperative Path Following example

"""


import numpy as np
import pickle
from math import pi

import pathgeneration as pg
import lib.sim2.systembuild as sb
import lib.sim2.plot as plotting


def simulation(file_name):
    if file_name != "":
        f = open("lib\sim2\\" + file_name + ".txt", 'wb')

    # Path parameters
    resolution = 40
    start = 0
    position0 = np.array([0, 0])
    position1 = np.array([0, 0])
    orientation = -pi/2
    size = 25.0
    arc = 2*pi
    radius = size

    # Path creation
    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position0, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position1, orientation, arc, radius - 5, start)
    p1.append_path(circle1)

    # Time parameters
    total_time = 200
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10
    y = -14
    theta_m = 0
    s = 0

    nominal_speed_profile = 0.5

    cpf_params = {
        "norm0": p0.total_distance,
        "norm1": p1.total_distance,
        "speed_profile0": nominal_speed_profile,
        "speed_profile1": nominal_speed_profile * p1.total_distance / p0.total_distance
    }

    # System creation along with initial conditions
    auv_pf_system = sb.DoubleAUVCPFContinuousCommunications(p0, p1, k_csi=0.1, cpf_params=cpf_params, gamma=1, k1=1, k2=0.3, k_delta=0.5, theta_a=0.8, history=True, dt=dt)
    ic = {"x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y + 5, "theta_m1": theta_m, "s1": s}
    auv_pf_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_pf_system.update(t)

    # Get past values for plotting
    past_values = auv_pf_system.past_values()

    paths = {"p0": p0, "p1": p1}

    if file_name != "":
        pickle.dump(paths, f)
        pickle.dump(num_points, f)
        pickle.dump(total_time, f)
        pickle.dump(resolution, f)
        pickle.dump(T, f)
        pickle.dump(past_values, f)
        f.close()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)