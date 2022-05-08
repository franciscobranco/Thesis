"""

Author: Francisco Branco
Created: 04/05/2022
Description: Plotting for Formation Control Example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values):
    p_target0 = paths["p_target0"]
    p_target1 = paths["p_target1"]
    p_target2 = paths["p_target2"]
    p_tracker0 = paths["p_tracker0"]
    p_tracker1 = paths["p_tracker1"]

    # Plotting
    all_outputs, kine_target0, pf_target0, ckf_target0, cpf_target0, kine_target1, pf_target1, ckf_target1, cpf_target1, cfc_target1, kine_target2, pf_target2, ckf_target2, cpf_target2, mpf_tracker0, pf_tracker0, cpf_tracker0, cfc_tracker0, cfc_centre, mpf_tracker1, pf_tracker1, cpf_tracker1, ekf_tracker = past_values

    #print("Broadcasts: " + str(len(cpf_target0["broadcasts"]) + len(cpf_target1["broadcasts"]) + len(cpf_target2["broadcasts"]) + len(cpf_tracker0["broadcasts"]) + len(cpf_tracker1["broadcasts"])))
    print("Broadcasts: " + str(len(cpf_tracker0["broadcasts"]) + len(cpf_tracker1["broadcasts"])))


    # Start plotting
    fig, ax = plt.subplots(2, 3, constrained_layout=True)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_factor = 8
    frame_rate = num_points / total_time * frame_factor
    
    
    """
    p_target0.plot_path(ax[0][0])
    p_target1.plot_path(ax[0][0])
    p_target2.plot_path(ax[0][0])

    ax[0][0].plot(all_outputs["x_target0"][-1], all_outputs["y_target0"][-1], color='tab:blue', marker='o')
    ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:orange', marker='o')
    ax[0][0].plot(all_outputs["x_target2"][-1], all_outputs["y_target2"][-1], color='tab:green', marker='o')

    p_r = pg.Path()
    circle_r = pg.Circle(resolution, np.array([all_outputs["x_target1"][-1], all_outputs["y_target1"][-1]]), all_outputs["theta_m_target1"][-1], arc_tracker, radius_tracker, start_tracker)
    p_r.append_path(circle_r)
    p_r.plot_path(ax[0][0])

    # Plot vehicle and past course
    ax[0][0].plot(all_outputs["x_tracker0"][-1], all_outputs["y_tracker0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][-1] / (2*pi) - 90), markersize=10)
    ax[0][0].plot(all_outputs["x_tracker1"][-1], all_outputs["y_tracker1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][-1] / (2*pi) - 90), markersize=10)
    ax[0][0].plot(all_outputs["x_ekf"][-1], all_outputs["y_ekf"][-1], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][-1] / (2*pi) - 90), markersize=10)
    
    ax[0][0].plot(all_outputs["x_tracker0"], all_outputs["y_tracker0"], color='magenta', linestyle='--')
    ax[0][0].plot(all_outputs["x_tracker1"], all_outputs["y_tracker1"], color='red', linestyle='--')
    ax[0][0].plot(all_outputs["x_ekf"], all_outputs["y_ekf"], color='purple', linestyle='--')

    ax[0][0].legend(['target path0', 'target path1', 'target path2', 'target0', 'target1', 'target2', 'moving path', 'tracker0', 'tracker1', 'estimate'], bbox_to_anchor=(0.75, 0.75))
    # Labels and grid
    ax[0][0].set_title('Position plot')
    ax[0][0].set_xlabel('X [m]')
    ax[0][0].set_ylabel('Y [m]')
    ax[0][0].grid()
    ax[0][0].axis('equal')
    

    # Velocity plot
    ax[1][0].set_title('Velocity plot')
    ax[1][0].plot(T, all_outputs["velocity_target0"])
    ax[1][0].plot(T, all_outputs["velocity_target1"])
    ax[1][0].plot(T, all_outputs["velocity_target2"])
    ax[1][0].plot(T, all_outputs["velocity_tracker0"], color='magenta', linestyle='-')
    ax[1][0].plot(T, all_outputs["velocity_tracker1"], color='red', linestyle='-')
    #ax[1].set_ylim([0.98, 1.02])
    ax[1][0].set_xlabel('time [s]')
    ax[1][0].set_ylabel('Velocity [m/s]')
    ax[1][0].legend(['target0', 'target1', 'target2', 'tracker0', 'tracker1'])
    ax[1][0].grid()



    # Range-measurement plot
    measurements0 = [[], []]
    measurements1 = [[], []]
    for j in range(len(T)):
        if all_outputs["range0"][j] >= 0:
            measurements0[0].append(T[j])
            measurements0[1].append(all_outputs["range0"][j])
        if all_outputs["range1"][j] >= 0:
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
    ax[1][1].plot(T, error)
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
    ax[1][2].grid()
    ax[0][2].legend(['tracker0 y1', 'tracker0 s1', 'tracker1 y1', 'tracker1 s1'])
    
    # Lapierre output u plot
    ax[1][2].set_title('Lapierre output u')
    ax[1][2].plot(T, all_outputs["u_target0"])
    ax[1][2].plot(T, all_outputs["u_target1"])
    ax[1][2].plot(T, all_outputs["u_target2"])
    ax[1][2].plot(T, all_outputs["u_tracker0"], color='magenta', linestyle='-')
    ax[1][2].plot(T, all_outputs["u_tracker1"], color='red', linestyle='-')
    ax[1][2].grid()
    ax[1][2].legend(['target0', 'target1', 'target2', 'tracker0', 'tracker1'])
    

    fig.show()
    plt.pause(100)

    """

    for i in range(len(T)):
        if i % frame_rate == 0:

            if i != len(T) - 1:
                ax[0][0].cla()
                ax[1][0].cla()
                ax[0][1].cla()
                ax[1][1].cla()
                ax[0][2].cla()
                ax[1][2].cla()

            
            ax[0][0].set_title('Position plot', y=1.0, pad=-14)
            p_target0.plot_path(ax[0][0])
            p_target1.plot_path(ax[0][0])
            p_target2.plot_path(ax[0][0])

            ax[0][0].plot(p_target0.get_x(pf_target0["s"][i]), p_target0.get_y(pf_target0["s"][i]), color='lightskyblue', marker='o', label='_nolegend_')
            ax[0][0].plot(p_target1.get_x(pf_target1["s"][i]), p_target1.get_y(pf_target1["s"][i]), color='orange', marker='o', label='_nolegend_')
            ax[0][0].plot(p_target2.get_x(pf_target2["s"][i]), p_target2.get_y(pf_target2["s"][i]), color='limegreen', marker='o', label='_nolegend_')
            ax[0][0].plot(p_target1.get_x(cfc_centre[i]) + p_tracker0.get_x(pf_tracker0["s"][i]), p_target1.get_y(cfc_centre[i]) + p_tracker0.get_y(pf_tracker0["s"][i]), color='violet', marker='o', label='_nolegend_')
            ax[0][0].plot(p_target1.get_x(cfc_centre[i]) + p_tracker1.get_x(pf_tracker1["s"][i]), p_target1.get_y(cfc_centre[i]) + p_tracker1.get_y(pf_tracker1["s"][i]), color='orangered', marker='o', label='_nolegend_')

            ax[0][0].plot(all_outputs["x_target0"][i], all_outputs["y_target0"][i], color='tab:blue', marker='o')
            ax[0][0].plot(all_outputs["x_target1"][i], all_outputs["y_target1"][i], color='tab:orange', marker='o')
            ax[0][0].plot(all_outputs["x_target2"][i], all_outputs["y_target2"][i], color='tab:green', marker='o')
            ax[0][0].plot(all_outputs["x_target0"][:i], all_outputs["y_target0"][:i], color='tab:blue', linestyle='--', label='_nolegend_')
            ax[0][0].plot(all_outputs["x_target1"][:i], all_outputs["y_target1"][:i], color='tab:orange', linestyle='--', label='_nolegend_')
            ax[0][0].plot(all_outputs["x_target2"][:i], all_outputs["y_target2"][:i], color='tab:green', linestyle='--', label='_nolegend_')

            # ax[0][0].plot(all_outputs["x_pred_target0"][i], all_outputs["y_pred_target0"][i], color='tab:cyan', marker='o')
            # ax[0][0].plot(all_outputs["x_pred_target1"][i], all_outputs["y_pred_target1"][i], color='tab:gray', marker='o')
            # ax[0][0].plot(all_outputs["x_pred_target2"][i], all_outputs["y_pred_target2"][i], color='tab:olive', marker='o')
            # ax[0][0].plot(all_outputs["x_pred_target0"][:i], all_outputs["y_pred_target0"][:i], color='tab:cyan', linestyle='--')
            # ax[0][0].plot(all_outputs["x_pred_target1"][:i], all_outputs["y_pred_target1"][:i], color='tab:gray', linestyle='--', label='_nolegend_')
            # ax[0][0].plot(all_outputs["x_pred_target2"][:i], all_outputs["y_pred_target2"][:i], color='tab:olive', linestyle='--')

            p_r = pg.Path()
            circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[i]), p_target1.get_y(cfc_centre[i])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
            p_r.append_path(circle_r)
            p_r.plot_path(ax[0][0])

            ax[0][0].plot(all_outputs["x_tracker0"][i], all_outputs["y_tracker0"][i], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][i] / (2*pi) - 90), markersize=10)
            ax[0][0].plot(all_outputs["x_tracker1"][i], all_outputs["y_tracker1"][i], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][i] / (2*pi) - 90), markersize=10)
            # ax[0][0].plot(all_outputs["x_ekf"][i], all_outputs["y_ekf"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][i] / (2*pi) - 90), markersize=10)
            
            #ax[0][0].plot(all_outputs["x_tracker0"][:i], all_outputs["y_tracker0"][:i], color='magenta', linestyle='--')
            #ax[0][0].plot(all_outputs["x_tracker1"][:i], all_outputs["y_tracker1"][:i], color='red', linestyle='--')
            #ax[0][0].plot(all_outputs["x_ekf"][:i], all_outputs["y_ekf"][:i], color='purple', linestyle='--')
            
            ax[0][0].legend([
                'target path0',
                'target path1',
                'target path2',
                'target0',
                'target1',
                # 'target1 prediction',
                'target2',
                'moving path',
                'tracker0',
                'tracker1',
                # 'EKF estimate'
                ],
                bbox_to_anchor=(0.75, 0.75), prop={'size': 6})

            # Labels and grid
            ax[0][0].set_xlabel('X [m]')
            ax[0][0].set_ylabel('Y [m]')
            ax[0][0].grid()
            ax[0][0].axis('equal')

            # Velocity plot
            ax[1][0].set_title('Velocity plot', y=1.0, pad=-14)
            ax[1][0].plot(T[:i], all_outputs["velocity_target0"][:i], color='tab:blue', linestyle='-')
            ax[1][0].plot(T[:i], all_outputs["velocity_target1"][:i], color='tab:orange', linestyle='-')
            ax[1][0].plot(T[:i], all_outputs["velocity_target2"][:i], color='tab:green', linestyle='-')
            ax[1][0].plot(T[:i], all_outputs["velocity_tracker0"][:i], color='magenta', linestyle='-')
            ax[1][0].plot(T[:i], all_outputs["velocity_tracker1"][:i], color='red', linestyle='-')
            # ax[1][0].plot(T[:i], mpf_tracker0["velocity"][:i], color='magenta', linestyle='-')
            # ax[1][0].plot(T[:i], mpf_tracker1["velocity"][:i], color='red', linestyle='-')
            ax[1][0].set_xlabel('time [s]')
            ax[1][0].set_ylabel('velocity [m/s]')
            ax[1][0].legend([
                'target0',
                'target1',
                'target2',
                'tracker0',
                'tracker1'], prop={'size': 6})
            ax[1][0].grid()

            # Cooperative Formation Control Plot
            ax[0][1].set_title('Cooperative Formation Control', y=1.0, pad=-14)
            ax[0][1].plot(T[:i], cfc_centre[:i], c='magenta')
            ax[0][1].plot(T[:i], pf_target1["s"][:i], c='tab:orange')
            broadcasts = [[], []]
            for j in range(len(cfc_tracker0["broadcasts"])):
                if cfc_tracker0["broadcasts"][j] <= T[i]:
                    broadcasts[0].append(cfc_tracker0["broadcasts"][j])
            for j in range(len(cfc_target1["broadcasts"])):
                if cfc_target1["broadcasts"][j] <= T[i]:
                    broadcasts[1].append(cfc_target1["broadcasts"][j])
            ax[0][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), -1), c='magenta', marker='+')
            ax[0][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), -1), c='tab:orange', marker='+')
            ax[0][1].set_xlabel('time [s]')
            ax[0][1].set_ylabel('gammas')
            ax[0][1].legend([
                'tracker0',
                'target1',
                'broadcast tracker',
                'broadcast target'], prop={'size': 6})
            ax[0][1].grid()

            """
            # Range-measurement plot
            ax[0][1].set_title('Range-measurements')
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
            ax[0][1].grid()
            """

            # ETC Broadcasting plot
            ax[1][1].set_title('ETC Broadcasting', y=1.0, pad=-14)
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
            for j in range(len(cpf_tracker0["broadcasts"])):
                if cpf_tracker0["broadcasts"][j] <= T[i]:
                    broadcasts[3].append(cpf_tracker0["broadcasts"][j])
            for j in range(len(cpf_tracker1["broadcasts"])):
                if cpf_tracker1["broadcasts"][j] <= T[i]:
                    broadcasts[4].append(cpf_tracker1["broadcasts"][j])

            ax[1][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), 0), c='tab:blue', marker='+')
            ax[1][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), 1), c='tab:orange', marker='+')
            ax[1][1].scatter(broadcasts[2], np.full(len(broadcasts[2]), 2), c='tab:green', marker='+')
            ax[1][1].scatter(broadcasts[3], np.full(len(broadcasts[3]), 3), c='magenta', marker='+')
            ax[1][1].scatter(broadcasts[4], np.full(len(broadcasts[4]), 4), c='red', marker='+')
            ax[1][1].set_xlim([0, T[i]])
            ax[1][1].set_xlabel('time [s]')
            ax[1][1].set_ylabel('Broadcasts')
            ax[1][1].legend([
                'target0',
                'target1',
                'target2',
                'tracker0',
                'tracker1'], prop={'size': 6})
            ax[1][1].grid()

            """
            # EKF error Plot
            error = []
            for j in range(i):
                error.append(np.sqrt(np.power(ekf_tracker["x"][j] - all_outputs["x_target1"][j], 2) + np.power(ekf_tracker["y"][j] - all_outputs["y_target1"][j], 2)))
            ax[1][1].plot(T[:i], error[:i], c='purple')
            
            position_error = []
            for j in range(i):
                position_error.append(np.sqrt(np.power(all_outputs["x_target1"][j] - all_outputs["x_pred_target1"][j], 2) + np.power(all_outputs["y_target1"][j] - all_outputs["y_pred_target1"][j], 2)))
            ax[1][1].plot(T[:i], position_error, c='tab:gray')
            
            ax[1][1].set_xlabel('time [s]')
            ax[1][1].set_ylabel('distance [m]')
            ax[1][1].legend(['EKF estimate', 'CKF estimate'])
            ax[1][1].set_title('EKF and CKF Error plot')
            ax[1][1].grid()
            """
            
            # Vehicle projection on path plot
            ax[0][2].set_title('Vehicle Postion on Path', y=1.0, pad=-14)
            ax[0][2].plot(T[:i], pf_target0["s"][:i], c='tab:blue')
            ax[0][2].plot(T[:i], pf_target1["s"][:i], c='tab:orange')
            ax[0][2].plot(T[:i], pf_target2["s"][:i], c='tab:green')
            ax[0][2].plot(T[:i], pf_tracker0["s"][:i], c='magenta')
            ax[0][2].plot(T[:i], pf_tracker1["s"][:i], c='red')
            ax[0][2].set_xlabel('time [s]')
            ax[0][2].set_ylabel('gammas')
            ax[0][2].legend([
                'target0',
                'target1',
                'target2',
                'tracker0',
                'tracker1'], prop={'size': 6})
            ax[0][2].grid()
            
            # Lapierre output u plot
            ax[1][2].set_title('Lapierre output u', y=1.0, pad=-14)
            ax[1][2].plot(T[:i], all_outputs["u_target0"][:i])
            ax[1][2].plot(T[:i], all_outputs["u_target1"][:i])
            ax[1][2].plot(T[:i], all_outputs["u_target2"][:i])
            ax[1][2].plot(T[:i], all_outputs["u_tracker0"][:i], color='magenta', linestyle='-')
            ax[1][2].plot(T[:i], all_outputs["u_tracker1"][:i], color='red', linestyle='-')
            ax[1][2].set_xlabel('time [s]')
            ax[1][2].set_ylabel('angle rate [rad/s]')
            ax[1][2].grid()
            ax[1][2].legend([
                'target0',
                'target1',
                'target2',
                'tracker0',
                'tracker1'], prop={'size': 6})


            

            fig.show()
            plt.pause(0.001)
            
    plt.pause(100)

    