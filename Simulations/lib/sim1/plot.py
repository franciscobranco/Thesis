"""

Author: Francisco Branco
Created: 24/08/2021

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt


def plot(paths, num_points, total_time, resolution, T, past_values):
    all_outputs, _, _, _ = past_values

    p1 = paths["p1"]
    
    # Start plotting
    fig, ax = plt.subplots(1,2)
    plt.ion()

    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 0.25
    frame_rate = num_points / total_time * frame_factor

    i = 0

    for i in range(len(T)):
        if i % frame_rate == 0:
            # Start by plotting vehicle position and trajectory
            p1.plot_path(ax[0])

            if i == 0:
                ax[0].set_title('AUV position plot')
                ax[0].set_xlabel('X [m]')
                ax[0].set_ylabel('Y [m]')
                ax[0].grid()

                fig.show()
                plt.pause(2)
                ax[0].cla()
                ax[1].cla()

            ax[0].plot(all_outputs["x"][i], all_outputs["y"][i], 'ro')

            X, Y = p1.get_xy(all_outputs["s"][i])
            ax[0].plot(X, Y, 'go')

            ax[0].plot(all_outputs["x"][:i], all_outputs["y"][:i], 'r--')

            ax[0].set_title('AUV position plot')
            ax[0].set_xlabel('X [m]')
            ax[0].set_ylabel('Y [m]')
            ax[0].grid()
            ax[0].legend(['Path', 'Vehicle', 'Virtual target'])


            # Plot Lapierre error
            error = []
            for j in range(i):
                error.append(np.sqrt(np.power(all_outputs["x"][j] - p1.get_xy(all_outputs["s"][j])[0], 2) + np.power(all_outputs["y"][j] - p1.get_xy(all_outputs["s"][j])[1], 2)))
            ax[1].plot(T[:i], error)
            ax[1].set_title('AUV Lapierre Error')
            ax[1].set_xlabel('time [s]')
            ax[1].set_ylabel('Distance between vehicle and virutal target')
            ax[1].grid()


            fig.show()
            plt.pause(0.01)

            if i != len(T) - 1:
                ax[0].cla()
                ax[1].cla()
            else:
                plt.pause(100)