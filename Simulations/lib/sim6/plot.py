"""

Author: Francisco Branco
Created: 17/11/2021
Description: Formation Control Simple example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf_target0, cpf_target0, _, pf_target1, cpf_target1, _, pf_target2, cpf_target2, _, pf_follower0, cpf_follower0, _, pf_follower1, cpf_follower1 = past_values

    p_target0 = paths["p_target0"]
    p_target1 = paths["p_target1"]
    p_target2 = paths["p_target2"]
    p_follower0 = paths["p_follower0"]
    p_follower1 = paths["p_follower1"]

    print("Target Broadcasts: " + str(len(cpf_target0["broadcasts"]) + len(cpf_target1["broadcasts"]) + len(cpf_target2["broadcasts"])))
    print("Tracker Broadcasts: " + str(len(cpf_follower0["broadcasts"]) + len(cpf_follower1["broadcasts"])))
    input("Press Enter to start plotting...")

    # Start plotting
    fig, ax = plt.subplots(2,2)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 1
    frame_rate = num_points / total_time * frame_factor

    Movie = False
    
    
    if Movie == False:
        p_target0.plot_path(ax[0][0])
        p_target1.plot_path(ax[0][0])
        p_target2.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_target0"], all_outputs["y_target0"], color='tab:blue', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_target1"], all_outputs["y_target1"], color='tab:orange', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_target2"], all_outputs["y_target2"], color='tab:green', linestyle='--', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_follower0"], all_outputs["y_follower0"], linestyle='--', color='magenta', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_follower1"], all_outputs["y_follower1"], linestyle='--', color='red', label='_nolegend_')

        p_r = pg.Path()
        circle_r = pg.Circle(resolution, np.array([all_outputs["x_target1"][-1], all_outputs["y_target1"][-1]]), all_outputs["theta_m_target1"][-1], p_follower0.path_list[0].arc, p_follower0.path_list[0].radius, p_follower0.path_list[0].start)
        p_r.append_path(circle_r)
        p_r.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_target0"][-1], all_outputs["y_target0"][-1], color='tab:blue', marker='o')
        ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:orange', marker='o')
        ax[0][0].plot(all_outputs["x_target2"][-1], all_outputs["y_target2"][-1], color='tab:green', marker='o')

        # Plot vehicle and past course
        ax[0][0].plot(all_outputs["x_follower0"][-1], all_outputs["y_follower0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_follower0"][-1] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x_follower1"][-1], all_outputs["y_follower1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_follower1"][-1] / (2*pi) - 90), markersize=10)

        

        # Plot the virtual target
        #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
        #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
        #ax[0][0].plot(X1, Y1, 'go')

        # Labels and grid
        ax[0][0].set_title('Vehicle Position')
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].axis('equal')
        ax[0][0].legend([
            'Target Path 0',
            'Target Path 1',
            'Target Path 2',
            'Virtual Circle',
            'Target 0',
            'Target 1',
            'Target 2',
            'Tracker 0',
            'Tracker 1'])#, bbox_to_anchor=(0.75, 0.75))

        # Velocity plot
        ax[1][0].set_title('Vehicle Velocity')
        ax[1][0].plot(T, all_outputs["velocity_target0"], linestyle='-', color='tab:blue')
        ax[1][0].plot(T, all_outputs["velocity_target1"], linestyle='-', color='tab:orange')
        ax[1][0].plot(T, all_outputs["velocity_target2"], linestyle='-', color='tab:green')
        ax[1][0].plot(T, all_outputs["velocity_follower0"], linestyle='-', color='magenta')
        ax[1][0].plot(T, all_outputs["velocity_follower1"], linestyle='-', color='red')
        #ax[1].set_ylim([0.98, 1.02])
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('velocity [m/s]')
        ax[1][0].legend(['Target 0', 'Target 1', 'Target 2', 'Tracker 0', 'Tracker 1'])
        ax[1][0].grid()

        # Error plot
        ax[0][1].set_title('Coordination States')
        # difference = []
        
        # for count in range(len(T)):
        #     if all_outputs["s_follower0"][count] >= 0 and all_outputs["s_follower0"][count] <= 0.25 and all_outputs["s_follower1"][count] >= 0.75 and all_outputs["s_follower1"][count] <= 1:
        #         difference.append(all_outputs["s_follower0"][count] + 1 - all_outputs["s_follower1"][count])
        #     elif all_outputs["s_follower1"][count] >= 0 and all_outputs["s_follower1"][count] <= 0.25 and all_outputs["s_follower0"][count] >= 0.75 and all_outputs["s_follower0"][count] <= 1:
        #         difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count] - 1)
        #     else:
        #         difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count])
        #         #print(all_outputs["s0"][count] - all_outputs["s1"][count])
        
        # ax[0][1].plot(T, difference)
        ax[0][1].plot(T, all_outputs["s_follower0"], linestyle='-', color='magenta')
        ax[0][1].plot(T, all_outputs["s_follower1"], linestyle='-', color='red')
        #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].set_ylabel('$\gamma$')
        ax[0][1].legend(['Tracker 0', 'Tracker 1'])
        ax[0][1].grid()

        # Broadcast plot
        ax[1][1].set_title('Vehicle Broadcasting')
        # broadcasts = [[], [], [], [], []]
        # for j in range(len(cpf_target0["broadcasts"])):
        #     if cpf_target0["broadcasts"][j] <= T[i]:
        #         broadcasts[0].append(cpf_target0["broadcasts"][j])
        # for j in range(len(cpf_target1["broadcasts"])):
        #     if cpf_target1["broadcasts"][j] <= T[i]:
        #         broadcasts[1].append(cpf_target1["broadcasts"][j])
        # for j in range(len(cpf_target2["broadcasts"])):
        #     if cpf_target2["broadcasts"][j] <= T[i]:
        #         broadcasts[2].append(cpf_target2["broadcasts"][j])
        # for j in range(len(cpf_follower0["broadcasts"])):
        #     if cpf_follower0["broadcasts"][j] <= T[i]:
        #         broadcasts[3].append(cpf_follower0["broadcasts"][j])
        # for j in range(len(cpf_follower1["broadcasts"])):
        #     if cpf_follower1["broadcasts"][j] <= T[i]:
        #         broadcasts[4].append(cpf_follower1["broadcasts"][j])
        ax[1][1].scatter(cpf_target0["broadcasts"], np.full(len(cpf_target0["broadcasts"]), 0), c='tab:blue', marker='+')
        ax[1][1].scatter(cpf_target1["broadcasts"], np.full(len(cpf_target1["broadcasts"]), 1), c='tab:orange', marker='+')
        ax[1][1].scatter(cpf_target2["broadcasts"], np.full(len(cpf_target2["broadcasts"]), 2), c='tab:green', marker='+')
        ax[1][1].scatter(cpf_follower0["broadcasts"], np.full(len(cpf_follower0["broadcasts"]), 3), c='magenta', marker='+')
        ax[1][1].scatter(cpf_follower1["broadcasts"], np.full(len(cpf_follower1["broadcasts"]), 4), c='red', marker='+')
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('Broadcasts')
        ax[1][1].legend(['Target 0', 'Target 1', 'Target 2', 'Tracker 0', 'Tracker 1'])
        ax[1][1].set_xlim([0, T[-1]])
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
        
        # # s1 y1 plot
        # ax[0][2].set_title('Lapierre s1 and y1')
        # ax[0][2].plot(T, pf_target0["y1_geo"])
        # ax[0][2].plot(T, pf_target0["s1_geo"])
        # ax[0][2].plot(T, pf_target1["y1_geo"])
        # ax[0][2].plot(T, pf_target1["s1_geo"])
        # ax[0][2].plot(T, pf_target2["y1_geo"])
        # ax[0][2].plot(T, pf_target2["s1_geo"])
        # ax[0][2].plot(T, pf_follower0["y1_geo"])
        # ax[0][2].plot(T, pf_follower0["s1_geo"])
        # ax[0][2].plot(T, pf_follower1["y1_geo"])
        # ax[0][2].plot(T, pf_follower1["s1_geo"])
        # ax[0][2].grid()
        # ax[0][2].legend(['target0 y1', 'target0 s1', 'target1 y1', 'target1 s1', 'target2 y1', 'target2 s1', 'tracker0 y1', 'tracker0 s1', 'tracker1 y1', 'tracker1 s1'])
        # # ax[1].set_title('Tracker PF Error')
        # # error = []
        # # for j in range(len(T)):
        # #     error.append(np.sqrt(np.power(all_outputs["x_tracker"][j] - all_outputs["x_target"][j] - p0.get_xy(all_outputs["s_tracker"][j])[0], 2) + np.power(all_outputs["y_tracker"][j] - all_outputs["y_target"][j] - p0.get_xy(all_outputs["s_tracker"][j])[1], 2)))
        # # ax[1].plot(T, error, color='magenta', linestyle='-')
        
        # # # ax[1][1].plot(T, difference)
        # # #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
        # # ax[1].set_xlabel('time [s]')
        # # ax[1].set_ylabel('Distance between tracker and the virtual target [m]')
        # # ax[1].grid()

        # # Lapierre output u plot
        # ax[1][2].set_title('Lapierre output u')
        # ax[1][2].plot(T, all_outputs["u_target0"])
        # ax[1][2].plot(T, all_outputs["u_target1"])
        # ax[1][2].plot(T, all_outputs["u_target2"])
        # ax[1][2].plot(T, all_outputs["u_follower0"])
        # ax[1][2].plot(T, all_outputs["u_follower1"])
        # ax[1][2].set_ylim([-4, 4])
        # ax[1][2].grid()
        # ax[1][2].legend(['target0', 'target1', 'target2', 'tracker0', 'tracker1'])
        

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
                    ax[1][2].cla()

                p_target0.plot_path(ax[0][0])
                p_target1.plot_path(ax[0][0])
                p_target2.plot_path(ax[0][0])

                ax[0][0].plot(all_outputs["x_target0"][i], all_outputs["y_target0"][i], color='tab:blue', marker='o')
                ax[0][0].plot(all_outputs["x_target1"][i], all_outputs["y_target1"][i], color='tab:orange', marker='o')
                ax[0][0].plot(all_outputs["x_target2"][i], all_outputs["y_target2"][i], color='tab:green', marker='o')

                p_r = pg.Path()
                circle_r = pg.Circle(resolution, np.array([all_outputs["x_target1"][i], all_outputs["y_target1"][i]]), all_outputs["theta_m_target1"][i], p_follower0.path_list[0].arc, p_follower0.path_list[0].radius, p_follower0.path_list[0].start)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x_follower0"][i], all_outputs["y_follower0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m_follower0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_follower1"][i], all_outputs["y_follower1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m_follower1"][i] / (2*pi) - 90), markersize=10)

                ax[0][0].plot(all_outputs["x_follower0"][:i], all_outputs["y_follower0"][:i], 'r--')
                ax[0][0].plot(all_outputs["x_follower1"][:i], all_outputs["y_follower1"][:i], 'm--')
                

                # Plot the virtual target
                #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
                #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                
                
                
                #ax[0][0].plot(X1, Y1, 'go')
                ax[0][0].legend(['target0 path', 'target1 path', 'target2 path','target0', 'target1', 'target2', 'moving path', 'follower0', 'follower1'], bbox_to_anchor=(0.75, 0.75))


                # Labels and grid
                ax[0][0].set_title('Position plot')
                ax[0][0].set_xlabel('X [m]')
                ax[0][0].set_ylabel('Y [m]')
                ax[0][0].grid()
                ax[0][0].axis('equal')

                # Velocity plot
                ax[1][0].set_title('Velocity plot')
                ax[1][0].plot(T[:i], all_outputs["velocity_target0"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity_target1"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity_target2"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity_follower0"][:i])
                ax[1][0].plot(T[:i], all_outputs["velocity_follower1"][:i])
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].legend(['target0', 'target1', 'target2', 'follower0', 'follower1'])
                ax[1][0].grid()

                # Error plot
                ax[0][1].set_title('Gamma Error plot for Followers')
                difference = []
                
                for count in range(i):
                    if all_outputs["s_follower0"][count] >= 0 and all_outputs["s_follower0"][count] <= 0.25 and all_outputs["s_follower1"][count] >= 0.75 and all_outputs["s_follower1"][count] <= 1:
                        difference.append(all_outputs["s_follower0"][count] + 1 - all_outputs["s_follower1"][count])
                    elif all_outputs["s_follower1"][count] >= 0 and all_outputs["s_follower1"][count] <= 0.25 and all_outputs["s_follower0"][count] >= 0.75 and all_outputs["s_follower0"][count] <= 1:
                        difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count] - 1)
                    else:
                        difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count])
                        #print(all_outputs["s0"][count] - all_outputs["s1"][count])
                
                ax[0][1].plot(T[:i], difference)
                ax[0][1].plot(T[:i], all_outputs["s_follower0"][:i])
                ax[0][1].plot(T[:i], all_outputs["s_follower1"][:i])
                #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
                ax[0][1].set_xlabel('time [s]')
                ax[0][1].legend(['difference', 'follower0 s', 'follower1 s'])
                ax[0][1].grid()

                # Broadcast plot
                ax[1][1].set_title('Broadcasting plot')
                broadcasts = [[], [], [], [], []]
                for j in range(len(cpf_target0["broadcasts"])):
                    if cpf_target0["broadcasts"][j] <= T[i]:
                        broadcasts[0].append(cpf_target0["broadcasts"][j])
                for j in range(len(cpf_target1["broadcasts"])):
                    if cpf_target1["broadcasts"][j] <= T[i]:
                        broadcasts[1].append(cpf_target1["broadcasts"][j])
                for j in range(len(cpf_target2["broadcasts"])):
                    if cpf_target2["broadcasts"][j] <= T[i]:
                        broadcasts[2].append(cpf_target2["broadcasts"][j])
                for j in range(len(cpf_follower0["broadcasts"])):
                    if cpf_follower0["broadcasts"][j] <= T[i]:
                        broadcasts[3].append(cpf_follower0["broadcasts"][j])
                for j in range(len(cpf_follower1["broadcasts"])):
                    if cpf_follower1["broadcasts"][j] <= T[i]:
                        broadcasts[4].append(cpf_follower1["broadcasts"][j])
                ax[1][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), 0), c='tab:blue', marker='+')
                ax[1][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), 1), c='tab:orange', marker='+')
                ax[1][1].scatter(broadcasts[2], np.full(len(broadcasts[2]), 2), c='tab:green', marker='+')
                ax[1][1].scatter(broadcasts[3], np.full(len(broadcasts[3]), 3), c='r', marker='+')
                ax[1][1].scatter(broadcasts[4], np.full(len(broadcasts[4]), 4), c='m', marker='+')
                ax[1][1].set_xlabel('time [s]')
                ax[1][1].legend(['target0', 'target1', 'target2', 'follower0', 'follower1'])
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
                ax[0][2].set_title('Lapierre s1 and y1')
                ax[0][2].plot(T[:i], pf_target0["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target0["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target1["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target1["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target2["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target2["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_follower0["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_follower0["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_follower1["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_follower1["s1_geo"][:i])
                ax[0][2].grid()
                ax[0][2].legend(['target0 y1', 'target0 s1', 'target1 y1', 'target1 s1', 'target2 y1', 'target2 s1', 'follower0 y1', 'follower0 s1', 'follower1 y1', 'follower1 s1'])
                
                # Lapierre output u plot
                ax[1][2].set_title('Lapierre output u')
                ax[1][2].plot(T[:i], all_outputs["u_target0"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_target1"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_target2"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_follower0"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_follower1"][:i])
                ax[1][2].grid()
                ax[1][2].legend(['target0', 'target1', 'target2', 'follower0', 'follower1'])


                

                fig.show()
                plt.pause(0.001)
        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 