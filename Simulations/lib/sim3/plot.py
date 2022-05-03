"""

Author: Francisco Branco
Created: 10/09/2021
Description: ETC Cooperative Path Following example

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.systembuild as sb


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf0, cpf0, _, pf1, cpf1 = past_values

    p0 = paths["p0"]
    p1 = paths["p1"]
    
    # Start plotting
    fig, ax1 = plt.subplots(2,3)
    plt.ion()
    
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 2
    frame_rate = num_points / total_time * frame_factor

    """
    # Print the whole simulation
    p0.plot_path(ax1[0][0])
    p1.plot_path(ax1[0][0])

    ax1[0][0].plot(all_outputs["x0"], all_outputs["y0"], 'r--')
    ax1[0][0].plot(all_outputs["x1"], all_outputs["y1"], 'm--')

    # Labels and grid
    ax1[0][0].set_title('AUV position plot')
    ax1[0][0].set_xlabel('X [m]')
    ax1[0][0].set_ylabel('Y [m]')
    ax1[0][0].grid()
    ax1[0][0].legend(['target\'s path', 'target', 'follower'])

    # Velocity plot
    ax1[1][0].set_title('AUV Velocity plot')
    ax1[1][0].plot(T, all_outputs["velocity0"])
    ax1[1][0].plot(T, all_outputs["velocity1"])
    #ax1[1].set_ylim([0.98, 1.02])
    ax1[1][0].set_xlabel('time [s]')
    ax1[1][0].set_ylabel('Velocity [m/s]')
    ax1[1][0].grid()
    ax1[1][0].legend(['target', 'follower'])


    # Error plot
    ax1[0][1].set_title('AUV Gamma Error plot')
    difference = []
    for count in range(len(all_outputs["s0"])):
        difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
    ax1[0][1].plot(T, difference)
    
    #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
    ax1[0][1].set_xlabel('time [s]')
    ax1[0][1].grid()

    # Broadcast plot
    ax1[1][1].set_title('AUV Broadcasting plot')
    ax1[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
    ax1[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
    ax1[1][1].set_xlabel('time [s]')
    ax1[1][1].legend(['target', 'follower'])

    
    # s1 y1 plot
    ax1[0][2].set_title('AUV s1 and y1')
    ax1[0][2].plot(T, pf0["y1_geo"])
    ax1[0][2].plot(T, pf0["s1_geo"])
    ax1[0][2].plot(T, pf1["y1_geo"])
    ax1[0][2].plot(T, pf1["s1_geo"])
    ax1[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])
    

    fig.show()
    plt.pause(100)


    """
    # Print frame by frame
    i = 0

    for i in range(len(T)):
        if i % frame_rate == 0:
            
            if i != len(T) - 1:
                ax1[0][0].cla()
                ax1[1][0].cla()
                ax1[0][1].cla()
                ax1[1][1].cla()
                ax1[0][2].cla()
        
            p0.plot_path(ax1[0][0])
            p1.plot_path(ax1[0][0])

            

            # Plot vehicle and past course
            ax1[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
            ax1[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

            ax1[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
            ax1[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')

            # Plot the virtual target
            # X0, Y0 = p0.get_xy(all_outputs["s0"][i])
            # X1, Y1 = p1.get_xy(all_outputs["s1"][i])
            # ax1[0][0].plot(X0, Y0, 'go')
            # ax1[0][0].plot(X1, Y1, 'go')


            # Labels and grid
            ax1[0][0].set_title('AUV position plot')
            ax1[0][0].set_xlabel('X [m]')
            ax1[0][0].set_ylabel('Y [m]')
            ax1[0][0].grid()
            ax1[0][0].legend(['vehicle0 path', 'vehicle1 path', 'vehicle0', 'vehicle1'])

            # Velocity plot
            ax1[1][0].set_title('AUV Velocity plot')
            ax1[1][0].plot(T[:i], all_outputs["velocity0"][:i])
            ax1[1][0].plot(T[:i], all_outputs["velocity1"][:i])
            #ax1[1].set_ylim([0.98, 1.02])
            ax1[1][0].set_xlabel('time [s]')
            ax1[1][0].set_ylabel('Velocity [m/s]')
            ax1[1][0].grid()
            ax1[1][0].legend(['target', 'follower'])

            # Error plot
            ax1[0][1].set_title('AUV Gamma Error plot')
            difference = []
            for count in range(i):
                difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
            ax1[0][1].plot(T[:i], difference)
            #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
            ax1[0][1].set_xlabel('time [s]')
            ax1[0][1].grid()

            # Broadcast plot
            ax1[1][1].set_title('AUV Broadcasting plot')
            broadcasts = [[], []]
            for j in range(len(cpf0["broadcasts"])):
                if cpf0["broadcasts"][j] <= T[i]:
                    broadcasts[0].append(cpf0["broadcasts"][j])
            for j in range(len(cpf1["broadcasts"])):
                if cpf1["broadcasts"][j] <= T[i]:
                    broadcasts[1].append(cpf1["broadcasts"][j])
            ax1[1][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), 0), c='r', marker='+')
            ax1[1][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), 1), c='m', marker='+')
            ax1[1][1].set_xlabel('time [s]')
            ax1[1][1].legend(['target', 'follower'])
            ax1[1][1].set_xlim([0, T[i]])
            ax1[1][1].grid()

            # Angle input plotting
            #ax1[2].plot(T[:i], all_outputs["u0"][:i])
            #ax1[2].plot(T[:i], all_outputs["u1"][:i])
            #ax1[2].set_xlabel('time [s]')
            #ax1[2].set_ylabel('Angle input')
            #ax1[2].grid()

            # Angle plotting
            #ax1[3].plot(T[:i], all_outputs["theta_m0"][:i])
            #ax1[3].plot(T[:i], all_outputs["theta_m1"][:i])
            #ax1[3].set_xlabel('time [s]')
            #ax1[3].set_ylabel('Angle [radians]')
            #ax1[3].grid()

            # s1 y1 plot
            ax1[0][2].set_title('AUV Lapierre s1 and y1')
            ax1[0][2].plot(T[:i], pf0["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf0["s1_geo"][:i])
            ax1[0][2].plot(T[:i], pf1["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf1["s1_geo"][:i])
            ax1[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])


            fig.show()
            plt.pause(0.001)

    plt.pause(100)