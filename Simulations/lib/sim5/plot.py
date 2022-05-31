"""

Author: Francisco Branco
Created: 04/11/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf_target, _, pf_tracker = past_values

    p_target = paths["p_target"]
    p0 = paths["p_tracker"]

    input("Press Enter to start plotting...")
    
    # Start plotting
    fig, ax = plt.subplots(1,2)
    plt.ion()
    
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 2
    frame_rate = num_points / total_time * frame_factor
    
    Movie = False
    
    if Movie == False:
        p_target.plot_path(ax[0])
        
        ax[0].plot(all_outputs["x_target"], all_outputs["y_target"], color='tab:blue', linestyle='--', label='_nolegend_')

        ax[0].plot(all_outputs["x_tracker"], all_outputs["y_tracker"], linestyle='--', color='magenta', label='_nolegend_')

        p_r = pg.Path()
        circle_r = pg.Circle(resolution, np.array([all_outputs["x_target"][-1], all_outputs["y_target"][-1]]), all_outputs["theta_m_target"][-1], p0.path_list[0].arc, p0.path_list[0].radius, p0.path_list[0].start)
        p_r.append_path(circle_r)
        p_r.plot_path(ax[0])

        ax[0].plot(all_outputs["x_target"][-1], all_outputs["y_target"][-1], color='tab:blue', marker='o')

        ax[0].plot(all_outputs["x_tracker"][-1], all_outputs["y_tracker"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker"][-1] / (2*pi) - 90), markersize=10)

        # Labels and grid
        ax[0].set_title('Vehicle Position')
        ax[0].set_xlabel('X [m]')
        ax[0].set_ylabel('Y [m]')
        ax[0].grid()
        ax[0].legend(['Target Path', 'Virtual Circle', 'Target', 'Tracker'])

        # # Velocity plot
        # ax[1][0].set_title('Vehicle Velocity')
        # ax[1][0].plot(T, all_outputs["velocity0"])
        # ax[1][0].plot(T, all_outputs["velocity1"])
        # #ax[1].set_ylim([0.98, 1.02])
        # ax[1][0].set_xlabel('time [s]')
        # ax[1][0].set_ylabel('velocity [m/s]')
        # ax[1][0].grid()
        # ax[1][0].legend(['Target', 'Tracker'])

        # Error plot
        # difference = []
        # for count in range(len(T)):
        #     if all_outputs["s0"][count] >= 0 and all_outputs["s0"][count] <= 0.25 and all_outputs["s1"][count] >= 0.75 and all_outputs["s1"][count] <= 1:
        #         difference.append(all_outputs["s0"][count] + 1 - all_outputs["s1"][count])
        #     elif all_outputs["s1"][count] >= 0 and all_outputs["s1"][count] <= 0.25 and all_outputs["s0"][count] >= 0.75 and all_outputs["s0"][count] <= 1:
        #         difference.append(all_outputs["s0"][count] - all_outputs["s1"][count] - 1)
        #     else:
        #         difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
        #         #print(all_outputs["s0"][count] - all_outputs["s1"][count])
        ax[1].set_title('Tracker PF Error')
        error = []
        for j in range(len(T)):
            error.append(np.sqrt(np.power(all_outputs["x_tracker"][j] - all_outputs["x_target"][j] - p0.get_xy(all_outputs["s_tracker"][j])[0], 2) + np.power(all_outputs["y_tracker"][j] - all_outputs["y_target"][j] - p0.get_xy(all_outputs["s_tracker"][j])[1], 2)))
        ax[1].plot(T, error, color='magenta', linestyle='-')
        
        # ax[1][1].plot(T, difference)
        #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax[1].set_xlabel('time [s]')
        ax[1].set_ylabel('Distance between tracker and the virtual target [m]')
        ax[1].grid()

        # # Lapierre output plot
        # ax[0][1].set_title('Vehicle PF Control Law')
        # ax[0][1].plot(T, all_outputs["u_target"], color='tab:blue', linestyle='-')
        # ax[0][1].plot(T, all_outputs["u_tracker"], color='magenta', linestyle='-')
        # ax[0][1].set_ylim([-4, 4])
        # ax[0][1].set_xlabel('time [s]')
        # ax[0][1].set_ylabel('angle rate [rad/s]')
        # ax[0][1].legend(['Target', 'Tracker'])
        # ax[0][1].grid()
        
        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...")
    if Movie == True:
        for i in range(len(T)):
            if i % frame_rate == 0:

                if i != len(T) - 1:
                    ax[0][0].cla()
                    ax[1][0].cla()
                    ax[0][1].cla()
                    ax[1][1].cla()
                    ax[0][2].cla()

                p_target.plot_path(ax[0][0])

                ax[0][0].plot(all_outputs["x_target"][i], all_outputs["y_target"][i], color='tab:blue', marker='o')

                p_r = pg.Path()
                circle_r = pg.Circle(resolution, np.array([all_outputs["x_target"][i], all_outputs["y_target"][i]]), all_outputs["theta_m_target"][i], p0.path_list[0].arc, p0.path_list[0].radius, p0.path_list[0].start)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

                ax[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
                ax[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')
                

                # Plot the virtual target
                #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
                #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                
                
                
                #ax[0][0].plot(X1, Y1, 'go')
                ax[0][0].legend(['target\'s path', 'target', 'followers\' path', 'follower0', 'follower1'])


                # Labels and grid
                ax[0][0].set_title('AUV position plot')
                ax[0][0].set_xlabel('X [m]')
                ax[0][0].set_ylabel('Y [m]')
                ax[0][0].grid()
                ax[0][0].axis('equal')

                # Velocity plot
                ax[1][0].set_title('AUV Velocity plot')
                ax[1][0].plot(T[:i], all_outputs["velocity0"][:i], color='r')
                ax[1][0].plot(T[:i], all_outputs["velocity1"][:i], color='m')
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].legend(['follower0', 'follower1'])
                ax[1][0].grid()

                # Error plot
                ax[0][1].set_title('Gamma Error plot between 2 Followers')
                difference = []
                
                for count in range(i):
                    if all_outputs["s0"][count] >= 0 and all_outputs["s0"][count] <= 0.25 and all_outputs["s1"][count] >= 0.75 and all_outputs["s1"][count] <= 1:
                        difference.append(all_outputs["s0"][count] + 1 - all_outputs["s1"][count])
                    elif all_outputs["s1"][count] >= 0 and all_outputs["s1"][count] <= 0.25 and all_outputs["s0"][count] >= 0.75 and all_outputs["s0"][count] <= 1:
                        difference.append(all_outputs["s0"][count] - all_outputs["s1"][count] - 1)
                    else:
                        difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
                        #print(all_outputs["s0"][count] - all_outputs["s1"][count])
                
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
                ax[1][1].legend(['follower0', 'follower1'])
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
                ax[0][2].legend(['follower0 y1', 'follower0 s1', 'follower1 y1', 'follower1 s1'])
                

                fig.show()
                plt.pause(0.001)
        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 