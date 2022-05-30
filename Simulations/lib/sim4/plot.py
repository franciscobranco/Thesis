"""
##############
#DISCONTINUED#
##############

Author: Francisco Branco
Created: 02/10/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values, factor):
    # Get past values for plotting
    all_outputs, _, pf0, cpf0, _, pf1, cpf1 = past_values

    p0 = paths["p0"]
    p1 = paths["p1"]
    
    print("Broadcasts: " + str(len(cpf0["broadcasts"]) + len(cpf1["broadcasts"])))
    input("Press Enter to start plotting...")

    
    # Start plotting
    fig, ax = plt.subplots(2,3)
    plt.ion()
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 8
    frame_rate = num_points / total_time * frame_factor

    Movie = False
    
    if Movie == False:
        p0.plot_path(ax[0][0])
        

        ax[0][0].plot(all_outputs["x0"], all_outputs["y0"], linestyle='--', color='tab:blue', label='_nolegend_')
        ax[0][0].plot(all_outputs["x1"], all_outputs["y1"], linestyle='--', color='magenta', label='_nolegend_')

        ax[0][0].plot(all_outputs["x0"], all_outputs["y0"], marker='o', color='tab:blue')
        ax[0][0].plot(all_outputs["x1"], all_outputs["y1"], marker='o', color='magenta')

        # Labels and grid
        ax[0][0].set_title('Vehicle Position')
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].legend(['target\'s path', 'target', 'tracker'])

        # Broadcast plot
        ax[1][1].set_title('Vehicle Broadcasting')
        ax[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='tab:blue', marker='+')
        ax[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='tab:orange', marker='+')
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('Broadcasts')
        ax[1][1].grid()
        ax[1][1].legend(['target', 'follower'])

        # Velocity plot
        ax[1][0].set_title('Vehicle Velocity')
        ax[1][0].plot(T, all_outputs["velocity0"])
        ax[1][0].plot(T, all_outputs["velocity1"])
        #ax[1].set_ylim([0.98, 1.02])
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('velocity [m/s]')
        ax[1][0].grid()
        ax[1][0].legend(['target', 'tracker'])



        # Error plot
        ax[0][1].set_title('Coordination Error')
        difference = []
        laps = 0
        for count in range(len(all_outputs["s0"])):
            if count != 0 and last_value > 0.9 + all_outputs["s1"][count]:
                laps = laps + 1
            difference.append(all_outputs["s0"][count] - (all_outputs["s1"][count] + laps) / factor)
            last_value = all_outputs["s1"][count]
        ax[0][1].plot(T, difference)
        
        #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].grid()


        
        # s1 y1 plot
        ax[0][2].set_title('AUV Lapierre s1 and y1')
        ax[0][2].plot(T, pf0["y1_geo"])
        ax[0][2].plot(T, pf0["s1_geo"])
        ax[0][2].plot(T, pf1["y1_geo"])
        ax[0][2].plot(T, pf1["s1_geo"])
        ax[0][2].legend(['target y1', 'target s1', 'follower y1', 'follower s1'])
        

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 

    """

    
    if Movie == True:
        for i in range(len(T)):
            if i % frame_rate == 0:

                if i != len(T) - 1:
                    ax[0][0].cla()
                    ax[1][0].cla()
                    ax[0][1].cla()
                    ax[1][1].cla()
                    ax[0][2].cla()

                p0.plot_path(ax[0][0])

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

                ax[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
                ax[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')
                

                # Plot the virtual target
                X0, Y0 = p0.get_xy(all_outputs["s0"][i])
                #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                ax[0][0].plot(X0, Y0, 'go')
                #ax[0][0].plot(X1, Y1, 'go')
                ax[0][0].legend(['target\'s path', 'target', 'follower'])


                # Labels and grid
                ax[0][0].set_title('AUV position plot')
                ax[0][0].set_xlabel('X [m]')
                ax[0][0].set_ylabel('Y [m]')
                ax[0][0].grid()
                ax[0][0].axis('equal')

                # Velocity plot
                ax[1][0].set_title('AUV Velocity plot')
                ax[1][0].plot(T[:i], all_outputs["velocity0"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity1"][:i])
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].grid()

                # Error plot
                ax[0][1].set_title('AUV Gamma Error plot')
                difference = []
                laps = 0
                for count in range(i):
                    if count != 0 and last_value > 0.9 + all_outputs["s1"][count]:
                        laps = laps + 1
                    difference.append(all_outputs["s0"][count] - (all_outputs["s1"][count] + laps) / factor)
                    last_value = all_outputs["s1"][count]

                
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
                ax[0][2].legend(['target y1', 'target s1', 'follower y1', 'follower s1'])
                

                fig.show()
                plt.pause(0.001)

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 