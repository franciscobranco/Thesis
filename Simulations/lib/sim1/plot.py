"""

Author: Francisco Branco
Created: 24/08/2021

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.systembuild as sb


def run_simulation(past_values, paths, T):
    all_outputs, _, _, _ = past_values

    p1 = paths["p1"]
    
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