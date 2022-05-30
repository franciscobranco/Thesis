"""

Author: Francisco Branco
Created: 17/02/2022
Description: Two ASVs single target pursuit with double range measurement example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf_target1, _, pf_tracker0, cpf_tracker0, _, pf_tracker1, cpf_tracker1, ekf_tracker = past_values
    

    p_target1 = paths["p_target1"]

    p_tracker0 = paths["p_tracker0"]
    p_tracker1 = paths["p_tracker1"]

    print("Tracker Broadcasts: " + str(len(cpf_tracker0["broadcasts"]) + len(cpf_tracker1["broadcasts"])))
    input("Press Enter to start plotting...")

    # Start plotting
    fig, ax = plt.subplots(2,3)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 1
    frame_rate = num_points / total_time * frame_factor
    
    Movie = False
    
    if Movie == False:
        p_target1.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:blue', linestyle='--', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_tracker0"], all_outputs["y_tracker0"], color='magenta', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_tracker1"], all_outputs["y_tracker1"], color='red', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_ekf"], all_outputs["y_ekf"], color='purple', linestyle='--', label='_nolegend_')

        p_r = pg.Path()
        circle_r = pg.Circle(resolution, np.array([all_outputs["x_target1"][-1], all_outputs["y_target1"][-1]]), all_outputs["theta_m_target1"][-1], p_tracker0.path_list[0].arc, p_tracker0.path_list[0].radius, p_tracker0.path_list[0].start)
        p_r.append_path(circle_r)
        p_r.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:blue', marker='o')

        # Plot vehicle and past course
        ax[0][0].plot(all_outputs["x_tracker0"][-1], all_outputs["y_tracker0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][-1] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x_tracker1"][-1], all_outputs["y_tracker1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][-1] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x_ekf"][-1], all_outputs["y_ekf"][-1], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][-1] / (2*pi) - 90), markersize=10)
        
        

        ax[0][0].legend(['target path', 'target1', 'virtual circle', 'tracker0', 'tracker1', 'estimate'], bbox_to_anchor=(0.75, 0.75))
        # Labels and grid
        ax[0][0].set_title('Vehicle Position')
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].axis('equal')
        

        # Velocity plot
        ax[1][0].set_title('Velocity plot')
        ax[1][0].plot(T, all_outputs["velocity_tracker0"], color='magenta', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_tracker1"], color='red', linestyle='-')
        #ax[1].set_ylim([0.98, 1.02])
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('Velocity [m/s]')
        ax[1][0].legend(['target1', 'tracker0', 'tracker1'])
        ax[1][0].grid()



        # Range-measurement plot
        measurements0 = [[], []]
        measurements1 = [[], []]
        for j in range(len(T)):
            if all_outputs["range0"][j] != None:
                measurements0[0].append(T[j])
                measurements0[1].append(all_outputs["range0"][j])
            if all_outputs["range1"][j] != None:
                measurements1[0].append(T[j])
                measurements1[1].append(all_outputs["range1"][j])
        ax[0][1].plot(measurements0[0], measurements0[1], color='magenta', linestyle='-')
        ax[0][1].plot(measurements1[0], measurements1[1], color='red', linestyle='-')
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].set_ylabel('distance measure [m]')
        ax[0][1].legend(['tracker0', 'tracker1'])
        ax[0][1].set_title('Range-measurement Plot')
        ax[0][1].grid()


        # EKF Velocity plot
        error = []
        for j in range(len(T)):
            error.append(np.sqrt(np.power(ekf_tracker["x"][j] - all_outputs["x_target1"][j], 2) + np.power(ekf_tracker["y"][j] - all_outputs["y_target1"][j], 2)))
        ax[1][1].plot(T, error, c='purple')
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('distance [m]')
        #ax[1][1].legend(['x_dot', 'y_dot', 'velocity'])
        ax[1][1].set_title('EKF Error plot')
        ax[1][1].grid()

        
        # s1 y1 plot
        ax[0][2].set_title('Trackers\' Lapierre s1 and y1')
        ax[0][2].plot(T, pf_tracker0["y1_geo"])
        ax[0][2].plot(T, pf_tracker0["s1_geo"])
        ax[0][2].plot(T, pf_tracker1["y1_geo"])
        ax[0][2].plot(T, pf_tracker1["s1_geo"])
        ax[0][2].grid()
        ax[0][2].legend(['tracker0 y1', 'tracker0 s1', 'tracker1 y1', 'tracker1 s1'])
        
        # Lapierre output u plot
        ax[1][2].set_title('Lapierre output u')
        ax[1][2].plot(T, all_outputs["u_target1"])
        ax[1][2].plot(T, all_outputs["u_tracker0"], color='magenta', linestyle='-')
        ax[1][2].plot(T, all_outputs["u_tracker1"], color='red', linestyle='-')
        ax[1][2].grid()
        ax[1][2].legend(['target1', 'tracker0', 'tracker1'])
        

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
                circle_r = pg.Circle(resolution, np.array([all_outputs["x_target1"][i], all_outputs["y_target1"][i]]), all_outputs["theta_m_target1"][i], arc_tracker, radius_tracker, start_tracker)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x_tracker0"][i], all_outputs["y_tracker0"][i], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_tracker1"][i], all_outputs["y_tracker1"][i], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_ekf"][i], all_outputs["y_ekf"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][i] / (2*pi) - 90), markersize=10)
                
                ax[0][0].plot(all_outputs["x_tracker0"][:i], all_outputs["y_tracker0"][:i], color='magenta', linestyle='--')
                ax[0][0].plot(all_outputs["x_tracker1"][:i], all_outputs["y_tracker1"][:i], color='red', linestyle='--')
                ax[0][0].plot(all_outputs["x_ekf"][:i], all_outputs["y_ekf"][:i], color='purple', linestyle='--')

                # Plot the virtual target
                #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
                #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                
                
                
                #ax[0][0].plot(X1, Y1, 'go')
                ax[0][0].legend(['target path0', 'target path1', 'target path2', 'target0', 'target1', 'target2', 'moving path', 'tracker0', 'tracker1', 'estimate'], bbox_to_anchor=(0.75, 0.75))


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
                ax[1][0].plot(T[:i], all_outputs["velocity_tracker0"][:i], color='magenta', linestyle='-')
                ax[1][0].plot(T[:i], all_outputs["velocity_tracker1"][:i], color='red', linestyle='-')
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].legend(['target0', 'target1', 'target2', 'tracker0', 'tracker1'])
                ax[1][0].grid()


                # Range-measurement plot
                measurements0 = [[], []]
                measurements1 = [[], []]
                for j in range(i):
                    if all_outputs["range0"][j] != None:
                        measurements0[0].append(T[j])
                        measurements0[1].append(all_outputs["range0"][j])
                    if all_outputs["range1"][j] != None:
                        measurements1[0].append(T[j])
                        measurements1[1].append(all_outputs["range1"][j])
                ax[0][1].plot(measurements0[0], measurements0[1], color='magenta', linestyle='-')
                ax[0][1].plot(measurements1[0], measurements1[1], color='red', linestyle='-')
                ax[0][1].set_xlabel('time [s]')
                ax[0][1].set_ylabel('distance measure [m]')
                ax[0][1].legend(['tracker0', 'tracker1'])
                ax[0][1].set_title('Range-measurement Plot')
                ax[0][1].grid()

                # EKF Velocity plot
                error = []
                for j in range(i):
                    error.append(np.sqrt(np.power(ekf_tracker["x"][j] - all_outputs["x_target1"][j], 2) + np.power(ekf_tracker["y"][j] - all_outputs["y_target1"][j], 2)))
                ax[1][1].plot(T[:i], error[:i])
                ax[1][1].set_xlabel('time [s]')
                ax[1][1].set_ylabel('distance [m]')
                #ax[1][1].legend(['x_dot', 'y_dot', 'velocity'])
                ax[1][1].set_title('EKF Error plot')
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
                ax[0][2].set_title('Trackers\' Lapierre s1 and y1')
                ax[0][2].plot(T[:i], pf_tracker0["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_tracker0["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_tracker1["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_tracker1["s1_geo"][:i])
                ax[0][2].grid()
                ax[0][2].legend(['tracker0 y1', 'tracker0 s1', 'tracker1 y1', 'tracker1 s1'])
                
                # Lapierre output u plot
                ax[1][2].set_title('Lapierre output u')
                ax[1][2].plot(T[:i], all_outputs["u_target0"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_target1"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_target2"][:i])
                ax[1][2].plot(T[:i], all_outputs["u_tracker0"][:i], color='magenta', linestyle='-')
                ax[1][2].plot(T[:i], all_outputs["u_tracker1"][:i], color='red', linestyle='-')
                ax[1][2].grid()
                ax[1][2].legend(['target0', 'target1', 'target2', 'tracker0', 'tracker1'])


                

                fig.show()
                plt.pause(0.001)
        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 