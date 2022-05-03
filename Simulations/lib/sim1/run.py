"""

Author: Francisco Branco
Created: 24/08/2021

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.sim1.systembuild as sb
import lib.sim1.plot as plotting


def run_simulation():
    # Path parameters
    resolution = 40
    start = 0
    position = np.array([10, 5])
    orientation = -pi/2
    size = 5.0
    arc = 2*pi
    radius = size

    # Path creation
    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position, orientation, arc, radius, start)
    p1.append_path(circle1)

    # Time parameters
    T, dt = np.linspace(start=0, stop=50, num=1000, retstep=True)

    # Vehicle initial conditions
    x = 5
    y = 0
    theta_m = 0
    s = 0


    # System creation along with initial conditions
    auv_pf_system = sb.SimpleAUVPathFollowing(some_path=p1, gamma=1, k1=1, k2=0.3, k_delta=1, theta_a=0.8, history=True, dt=dt)
    ic = {"x": x, "y": y, "theta_m": theta_m, "s": s}
    auv_pf_system.set_initial_conditions(ic)

    # Setup external inputs
    velocity = 1
    velocity_dot = 0
    inputs = {"velocity": velocity, "velocity_dot": velocity_dot}

    # Run the system
    for t in T:
        auv_pf_system.update(t, inputs)

    # Get past values for plotting
    past_values = auv_pf_system.past_values()

    paths = {"p1": p1}

    # Plotting
    plotting.plot(past_values, paths, T)