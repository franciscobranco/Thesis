"""

Author: Francisco Branco
Created: 10/09/2021
Description: ETC Cooperative Path Following example

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import utils


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf0, cpf0, _, pf1, cpf1 = past_values

    print("Broadcasts: " + str(len(cpf0["broadcasts"]) + len(cpf1["broadcasts"])))
    input("Press Enter to start plotting...")

    p0 = paths["p0"]
    p1 = paths["p1"]
    
    # Start plotting
    fig, ax = plt.subplots(2,2)
    plt.ion()
    
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 2
    frame_rate = num_points / total_time * frame_factor

    Movie = False

    if Movie == False:

        # Print the whole simulation
        p0.plot_path(ax[0][0])
        p1.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x0"], all_outputs["y0"], linestyle='--', color='tab:blue', label='_nolegend_')
        ax[0][0].plot(all_outputs["x1"], all_outputs["y1"], linestyle='--', color='tab:orange', label='_nolegend_')

        ax[0][0].plot(all_outputs["x0"][-1], all_outputs["y0"][-1], marker='o', color='tab:blue')
        ax[0][0].plot(all_outputs["x1"][-1], all_outputs["y1"][-1], marker='o', color='tab:orange')

        # Labels and grid
        ax[0][0].set_title('Vehicle Position')
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].legend(['Vehicle Path 0', 'Vehicle Path 1', 'Vehicle 0', 'Vehicle 1'])

        # Velocity plot
        ax[1][0].set_title('Vehicle Velocity')
        ax[1][0].plot(T, all_outputs["velocity0"])
        ax[1][0].plot(T, all_outputs["velocity1"])
        #ax[1].set_ylim([0.98, 1.02])
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('velocity [m/s]')
        ax[1][0].grid()
        ax[1][0].legend(['Vehicle 0', 'Vehicle 1'])


        # Error plot
        ax[0][1].set_title('Coordination Error')
        difference = []
        for count in range(len(all_outputs["s0"])):
            difference.append(utils.gamma_difference(all_outputs["s0"][count], all_outputs["s1"][count]))
        ax[0][1].plot(T, difference)
        
        #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].set_ylabel('$\gamma$ difference')
        ax[0][1].grid()

        # Broadcast plot
        ax[1][1].set_title('Vehicle Broadcasting')
        ax[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), color='tab:blue', marker='+')
        ax[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), color='tab:orange', marker='+')
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('Broadcasts')
        ax[1][1].set_xlim([0, T[-1]])
        ax[1][1].grid()
        ax[1][1].legend(['Vehicle 0', 'Vehicle 1'])

        
        # # s1 y1 plot
        # ax[0][2].set_title('AUV s1 and y1')
        # ax[0][2].plot(T, pf0["y1_geo"])
        # ax[0][2].plot(T, pf0["s1_geo"])
        # ax[0][2].plot(T, pf1["y1_geo"])
        # ax[0][2].plot(T, pf1["s1_geo"])
        # ax[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])
        

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 

    if Movie == True:
        # Print frame by frame

        for i in range(len(T)):
            if i % frame_rate == 0:
                
                if i != len(T) - 1:
                    ax[0][0].cla()
                    ax[1][0].cla()
                    ax[0][1].cla()
                    ax[1][1].cla()
                    ax[0][2].cla()
            
                p0.plot_path(ax[0][0])
                p1.plot_path(ax[0][0])

                

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

                ax[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
                ax[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')

                # Plot the virtual target
                # X0, Y0 = p0.get_xy(all_outputs["s0"][i])
                # X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                # ax[0][0].plot(X0, Y0, 'go')
                # ax[0][0].plot(X1, Y1, 'go')


                # Labels and grid
                ax[0][0].set_title('AUV position plot')
                ax[0][0].set_xlabel('X [m]')
                ax[0][0].set_ylabel('Y [m]')
                ax[0][0].grid()
                ax[0][0].legend(['vehicle0 path', 'vehicle1 path', 'vehicle0', 'vehicle1'])

                # Velocity plot
                ax[1][0].set_title('AUV Velocity plot')
                ax[1][0].plot(T[:i], all_outputs["velocity0"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity1"][:i])
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].grid()
                ax[1][0].legend(['target', 'follower'])

                # Error plot
                ax[0][1].set_title('AUV Gamma Error plot')
                difference = []
                for count in range(i):
                    difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
                ax[0][1].plot(T[:i], difference)
                #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
                ax[0][1].set_xlabel('time [s]')
                ax[0][1].grid()

                # Broadcast plot
                ax[1][1].set_title('AUV Broadcasting plot')
                broadcasts = [[], []]
                for j in range(len(cpf0["broadcasts"])):
                    if cpf0["broadcasts"][j] <= T[i]:
                        broadcasts[0].append(cpf0["broadcasts"][j])
                for j in range(len(cpf1["broadcasts"])):
                    if cpf1["broadcasts"][j] <= T[i]:
                        broadcasts[1].append(cpf1["broadcasts"][j])
                ax[1][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), 0), c='r', marker='+')
                ax[1][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), 1), c='m', marker='+')
                ax[1][1].set_xlabel('time [s]')
                ax[1][1].legend(['target', 'follower'])
                ax[1][1].set_xlim([0, T[i]])
                ax[1][1].grid()

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

                # s1 y1 plot
                ax[0][2].set_title('AUV Lapierre s1 and y1')
                ax[0][2].plot(T[:i], pf0["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf0["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf1["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf1["s1_geo"][:i])
                ax[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])


                fig.show()
                plt.pause(0.001)

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 