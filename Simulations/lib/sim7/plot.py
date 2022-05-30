"""

Author: Francisco Branco
Created: 17/02/2022
Description: Target Pursuit Simple example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    # Get past values for plotting
    all_outputs, _, pf_target, _, pf_tracker, ekf_tracker = past_values

    p_target = paths["p_target"]
    p_tracker = paths["p_tracker"]

    input("Press Enter to start plotting...")
    
    # Start plotting
    fig, ax = plt.subplots(2,3)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 8
    frame_rate = num_points / total_time * frame_factor
    
    """
    
    p0.plot_path(ax[0][0])
    

    ax[0][0].plot(all_outputs["x0"], all_outputs["y0"], 'r--')
    ax[0][0].plot(all_outputs["x1"], all_outputs["y1"], 'm--')

    # Labels and grid
    ax[0][0].set_title('AUV position plot')
    ax[0][0].set_xlabel('X [m]')
    ax[0][0].set_ylabel('Y [m]')
    ax[0][0].grid()
    ax[0][0].legend(['target\'s path', 'target', 'follower'])

    # Broadcast plot
    ax[1][1].set_title('AUV Broadcasting plot')
    ax[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
    ax[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
    ax[1][1].set_xlabel('time [s]')
    ax[1][1].legend(['target', 'follower'])

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
    plt.pause(100)

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
                    ax[1][2].cla()

                p_target.plot_path(ax[0][0])

                ax[0][0].plot(all_outputs["x_target"][i], all_outputs["y_target"][i], color='tab:blue', marker='o')

                p_r = pg.Path()
                circle_r = pg.Circle(resolution, np.array([all_outputs["x_target"][i], all_outputs["y_target"][i]]), all_outputs["theta_m_target"][i], p_tracker.path_list[0].arc, p_tracker.path_list[0].radius, p_tracker.path_list[0].start)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                # Plot vehicle and past course
                ax[0][0].plot(all_outputs["x_tracker"][i], all_outputs["y_tracker"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m_tracker"][i] / (2*pi) - 90), markersize=10)
                
                
                ax[0][0].plot(all_outputs["x_ekf"][i], all_outputs["y_ekf"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_tracker"][:i], all_outputs["y_tracker"][:i], 'r--')
                ax[0][0].plot(all_outputs["x_ekf"][:i], all_outputs["y_ekf"][:i], color='purple', linestyle='--')

                # Plot the virtual target
                #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
                #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
                
                
                
                #ax[0][0].plot(X1, Y1, 'go')
                ax[0][0].legend(['target path','target', 'moving path', 'tracker', 'estimate'], bbox_to_anchor=(0.75, 0.75))


                # Labels and grid
                ax[0][0].set_title('Position plot')
                ax[0][0].set_xlabel('X [m]')
                ax[0][0].set_ylabel('Y [m]')
                ax[0][0].grid()
                ax[0][0].axis('equal')

                # Velocity plot
                ax[1][0].set_title('Velocity plot')
                ax[1][0].plot(T[:i], all_outputs["velocity_target"][:i], c='tab:blue')
                ax[1][0].plot(T[:i], all_outputs["velocity_tracker"][:i], c='r')
                #ax[1].set_ylim([0.98, 1.02])
                ax[1][0].set_xlabel('time [s]')
                ax[1][0].set_ylabel('Velocity [m/s]')
                ax[1][0].legend(['target', 'tracker'])
                ax[1][0].grid()

                
                # # Error plot
                # ax[0][1].set_title('Gamma Error plot for Trackers')
                # difference = []
                
                # for count in range(i):
                #     if all_outputs["s_follower0"][count] >= 0 and all_outputs["s_follower0"][count] <= 0.25 and all_outputs["s_follower1"][count] >= 0.75 and all_outputs["s_follower1"][count] <= 1:
                #         difference.append(all_outputs["s_follower0"][count] + 1 - all_outputs["s_follower1"][count])
                #     elif all_outputs["s_follower1"][count] >= 0 and all_outputs["s_follower1"][count] <= 0.25 and all_outputs["s_follower0"][count] >= 0.75 and all_outputs["s_follower0"][count] <= 1:
                #         difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count] - 1)
                #     else:
                #         difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count])
                #         #print(all_outputs["s0"][count] - all_outputs["s1"][count])
                
                # ax[0][1].plot(T[:i], difference)
                # ax[0][1].plot(T[:i], all_outputs["s_follower0"][:i])
                # ax[0][1].plot(T[:i], all_outputs["s_follower1"][:i])
                # #ax[0][1].plot(T[:i], all_outputs["s1"][:i])
                # ax[0][1].set_xlabel('time [s]')
                # ax[0][1].legend(['difference', 'follower0 s', 'follower1 s'])
                # ax[0][1].grid()
                
                measurements = [[], []]
                # Range-measurement plot
                for j in range(i):
                    if all_outputs["range"][j] != None:
                        measurements[0].append(T[j])
                        measurements[1].append(all_outputs["range"][j])
                ax[0][1].plot(measurements[0], measurements[1])
                ax[0][1].set_title('Range Measurements')
                ax[0][1].set_xlabel('time [s]')
                ax[0][1].set_ylabel('distance measure [m]')
                #ax[0][1].legend(['difference', 'follower0 s', 'follower1 s'])
                #ax[0][1].title('Range-measurement Plot')
                ax[0][1].grid()

                # EKF Velocity plot
                #ax[1][1].plot(T[:i], ekf_tracker["x_dot"][:i])
                #ax[1][1].plot(T[:i], ekf_tracker["y_dot"][:i])
                #ax[1][1].plot(T[:i], all_outputs["velocity_ekf"][:i])
                #ax[1][1].set_xlabel('time [s]')
                #ax[1][1].set_ylabel('velocity [m/s]')
                #ax[1][1].legend(['x_dot', 'y_dot', 'velocity'])
                #ax[1][1].title('EKF Velocity plot')
                #ax[1][1].grid()
                error = []
                for j in range(i):
                    error.append(np.sqrt(np.power(ekf_tracker["x"][j] - all_outputs["x_target"][j], 2) + np.power(ekf_tracker["y"][j] - all_outputs["y_target"][j], 2)))

                ax[1][1].plot(T[:i], error[:i], c='purple')
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
                ax[0][2].set_title('Lapierre s1 and y1')
                ax[0][2].plot(T[:i], pf_target["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_target["s1_geo"][:i])
                ax[0][2].plot(T[:i], pf_tracker["y1_geo"][:i])
                ax[0][2].plot(T[:i], pf_tracker["s1_geo"][:i])
                ax[0][2].grid()
                ax[0][2].legend(['target y1', 'target s1', 'tracker y1', 'tracker s1'])
                
                # Lapierre output u plot
                ax[1][2].set_title('Lapierre output u')
                ax[1][2].plot(T[:i], all_outputs["u_target"][:i], c='tab:blue')
                ax[1][2].plot(T[:i], all_outputs["u_tracker"][:i], c='r')
                ax[1][2].grid()
                ax[1][2].legend(['target', 'tracker'])


                

                fig.show()
                plt.pause(0.001)
        fig.show()
        plt.pause(0.1)
