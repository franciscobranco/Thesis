"""

Author: Francisco Branco
Created: 24/08/2021

"""


#import sys
#import utils
import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.systembuild as sb


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
    all_outputs, _, _, _ = auv_pf_system.past_values()
    
    # Start plotting
    fig, ax1 = plt.subplots()
    plt.ion()
    fig.set_size_inches((7, 7))

    i = 0

    for i in range(len(T)):
        p1.plot_path(ax1)

        if i == 0:
            ax1.set_title('AUV position plot')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.grid()

            fig.show()
            plt.pause(2.5)
            ax1.cla()

        ax1.plot(all_outputs["x"][i], all_outputs["y"][i], 'ro')

        X, Y = p1.get_xy(all_outputs["s"][i])
        ax1.plot(X, Y, 'go')

        ax1.plot(all_outputs["x"][:i], all_outputs["y"][:i], 'r--')

        ax1.set_title('AUV position plot')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.grid()

        fig.show()
        plt.pause(0.01)

        if i != len(T) - 1:
            ax1.cla()
        else:
            plt.pause(100)