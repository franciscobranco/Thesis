"""

Author: Francisco Branco
Created: 24/08/2021
Description: Continuous Communication Cooperative Path Following example

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf0, _, pf1 = past_values

    p0 = paths["p0"]
    p1 = paths["p1"]
    
    # Start plotting
    fig, ax = plt.subplots(2,2)
    plt.ion()
    fig.set_size_inches((7, 7))

    
    p0.plot_path(ax[0][0])
    p1.plot_path(ax[0][0])

    ax[0][0].plot(all_outputs["x0"], all_outputs["y0"], 'r--')
    ax[0][0].plot(all_outputs["x1"], all_outputs["y1"], 'm--')

    # Labels and grid
    ax[0][0].set_title('AUV position plot')
    ax[0][0].set_xlabel('X [m]')
    ax[0][0].set_ylabel('Y [m]')
    ax[0][0].grid()
    ax[0][0].legend(['target\'s path', 'follower\'s path', 'target', 'follower'])

    # Velocity plot
    ax[1][0].set_title('AUV Velocity plot')
    ax[1][0].plot(T, all_outputs["velocity0"])
    ax[1][0].plot(T, all_outputs["velocity1"])
    #ax[1].set_ylim([0.98, 1.02])
    ax[1][0].set_xlabel('time [s]')
    ax[1][0].set_ylabel('Velocity [m/s]')
    ax[1][0].grid()
    ax[1][0].legend(['target', 'follower'])


    # Error plot
    ax[0][1].set_title('AUV Gamma Error plot')
    difference = []
    for count in range(len(all_outputs["s0"])):
        difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
    ax[0][1].plot(T, difference)
    
    #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
    ax[0][1].set_xlabel('time [s]')
    ax[0][1].grid()
    

    
    # s1 y1 plot
    ax[1][1].set_title('AUV s1 and y1')
    ax[1][1].plot(T, pf0["y1_geo"])
    ax[1][1].plot(T, pf0["s1_geo"])
    ax[1][1].plot(T, pf1["y1_geo"])
    ax[1][1].plot(T, pf1["s1_geo"])
    ax[1][1].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])
    

    fig.show()
    plt.pause(100)

    """

    i = 0

    for i in range(len(T)):
        p0.plot_path(ax[0][0])
        p1.plot_path(ax[0][0])


        # Plot vehicle and past course
        ax[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], 'r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], 'm', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

        ax[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
        ax[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')

        # Plot the virtual target
        X0, Y0 = p0.get_xy(all_outputs["s0"][i])
        X1, Y1 = p1.get_xy(all_outputs["s1"][i])
        ax[0][0].plot(X0, Y0, 'go')
        ax[0][0].plot(X1, Y1, 'go')

        # Labels and grid
        ax[0][0].set_title('AUV position plot')
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].legend(['vehicle1 path', 'vehicle2 path', 'vehicle1', 'vehicle2'])

        # Velocity plot
        ax[1][0].plot(T[:i], all_outputs["velocity0"][:i])
        ax[1][0].plot(T[:i], all_outputs["velocity1"][:i])
        #ax[1].set_ylim([0.98, 1.02])
        ax[1][0].set_title('AUV Velocity')
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('Velocity [m/s]')
        ax[1][0].grid()
        ax[1][0].legend(['target', 'follower'])

        # Angle input plotting
        #ax[2].plot(T[:i], all_outputs["u0"][:i])
        #ax[2].plot(T[:i], all_outputs["u1"][:i])
        #ax[2].set_xlabel('time [s]')
        #ax[2].set_ylabel('Angle input')
        #ax[2].grid()

        # Angle plotting
        #ax[3].plot(T[:i], all_outputs["theta_m0"][:i])
        #ax[3].plot(T[:i], all_outputs["theta_m1"][:i])
        #ax[3].set_xlabel('time [s]')
        #ax[3].set_ylabel('Angle [radians]')
        #ax[3].grid()

        # Error plot
        ax[0][1].set_title('AUV Gamma Error plot')
        difference = []
        for count in range(i):
            difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
        ax[0][1].plot(T[:i], difference[:i])
        
        #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].grid()

        # s1 y1 plot
        ax[1][1].set_title('AUV s1 and y1')
        ax[1][1].plot(T[:i], pf0["y1_geo"][:i])
        ax[1][1].plot(T[:i], pf0["s1_geo"][:i])
        ax[1][1].plot(T[:i], pf1["y1_geo"][:i])
        ax[1][1].plot(T[:i], pf1["s1_geo"][:i])
        ax[1][1].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])


        fig.show()
        plt.pause(0.001)

        if i != len(T) - 1:
            ax[0][0].cla()
            ax[1][0].cla()
            ax[0][1].cla()
            ax[1][1].cla()
        else:
            plt.pause(100)
    """