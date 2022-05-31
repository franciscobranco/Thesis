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
    all_outputs, kine_target0, pf_target0, cpf_target0, ckf_target0, kine_target1, pf_target1, cpf_target1, cfc_target1, ckf_target0, kine_target2, pf_target2, cpf_target2, ckf_target0, mpf_tracker0, pf_tracker0, cpf_tracker0, cfc_tracker0, cfc_centre, mpf_tracker1, pf_tracker1, cpf_tracker1, ekf_target0, ekf_target1, ekf_target2 = past_values

    print("Target Broadcasts: " + str(len(cpf_target0["broadcasts"]) + len(cpf_target1["broadcasts"]) + len(cpf_target2["broadcasts"])))
    print("Tracker Broadcasts: " + str(len(cpf_tracker0["broadcasts"]) + len(cpf_tracker1["broadcasts"])))
    print("Formation Broadcasts: " + str(len(cfc_tracker0["broadcasts"]) + len(cfc_target1["broadcasts"])))
    input("Press Enter to start plotting...")

    # Start plotting
    fig, ax = plt.subplots(3, 2)#, constrained_layout=True)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    Movie = False

    frame_factor = 16
    frame_rate = num_points / total_time * frame_factor

    legend_size = 8
    
    if Movie == False:
        ax[0][0].set_title('Vehicle Position')
        p_target0.plot_path(ax[0][0])
        p_target1.plot_path(ax[0][0])
        p_target2.plot_path(ax[0][0])

        ax[0][0].plot(p_target0.get_x(pf_target0["s"][-1]), p_target0.get_y(pf_target0["s"][-1]), color='lightskyblue', marker='o', label='_nolegend_')
        ax[0][0].plot(p_target1.get_x(pf_target1["s"][-1]), p_target1.get_y(pf_target1["s"][-1]), color='orange', marker='o', label='_nolegend_')
        ax[0][0].plot(p_target2.get_x(pf_target2["s"][-1]), p_target2.get_y(pf_target2["s"][-1]), color='limegreen', marker='o', label='_nolegend_')
        ax[0][0].plot(p_target1.get_x(cfc_centre[-1]) + p_tracker0.get_x(pf_tracker0["s"][-1]), p_target1.get_y(cfc_centre[-1]) + p_tracker0.get_y(pf_tracker0["s"][-1]), color='violet', marker='o', label='_nolegend_')
        ax[0][0].plot(p_target1.get_x(cfc_centre[-1]) + p_tracker1.get_x(pf_tracker1["s"][-1]), p_target1.get_y(cfc_centre[-1]) + p_tracker1.get_y(pf_tracker1["s"][-1]), color='orangered', marker='o', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_target0"], all_outputs["y_target0"], color='tab:blue', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_target1"], all_outputs["y_target1"], color='tab:orange', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_target2"], all_outputs["y_target2"], color='tab:green', linestyle='--', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_tracker0"], all_outputs["y_tracker0"], color='magenta', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_tracker1"], all_outputs["y_tracker1"], color='red', linestyle='--', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_pred_target0"], all_outputs["y_pred_target0"], color='tab:gray', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_pred_target1"], all_outputs["y_pred_target1"], color='tab:gray', linestyle='--', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_pred_target2"], all_outputs["y_pred_target2"], color='tab:gray', linestyle='--', label='_nolegend_')

        p_r = pg.Path()
        circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[-1]), p_target1.get_y(cfc_centre[-1])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
        p_r.append_path(circle_r)
        p_r.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_ekf0"][-1], all_outputs["y_ekf0"][-1], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf0"][-1] / (2*pi) - 90), markersize=11)
        ax[0][0].plot(all_outputs["x_ekf1"][-1], all_outputs["y_ekf1"][-1], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf1"][-1] / (2*pi) - 90), markersize=11, label='_nolegend_')
        ax[0][0].plot(all_outputs["x_ekf2"][-1], all_outputs["y_ekf2"][-1], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf2"][-1] / (2*pi) - 90), markersize=11, label='_nolegend_')

        ax[0][0].plot(all_outputs["x_target0"][-1], all_outputs["y_target0"][-1], color='tab:blue', marker='o')
        ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:orange', marker='o')
        ax[0][0].plot(all_outputs["x_target2"][-1], all_outputs["y_target2"][-1], color='tab:green', marker='o')

        ax[0][0].plot(all_outputs["x_pred_target0"][-1], all_outputs["y_pred_target0"][-1], color='tab:gray', marker='o')
        ax[0][0].plot(all_outputs["x_pred_target1"][-1], all_outputs["y_pred_target1"][-1], color='tab:gray', marker='o', label='_nolegend_')
        ax[0][0].plot(all_outputs["x_pred_target2"][-1], all_outputs["y_pred_target2"][-1], color='tab:gray', marker='o', label='_nolegend_')

        ax[0][0].plot(all_outputs["x_tracker0"][-1], all_outputs["y_tracker0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][-1] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x_tracker1"][-1], all_outputs["y_tracker1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][-1] / (2*pi) - 90), markersize=10)
        
        ax[0][0].legend([
            'Target Path 0',
            'Target Path 1',
            'Target Path 2',
            'Virtual Circle',
            'EKF Estimation',
            'Target 0',
            'Target 1',
            'Target 2',
            'CKF Prediction',
            'Tracker 0',
            'Tracker 1'
            ], prop={'size': legend_size})

        # Labels and grid
        ax[0][0].set_xlabel('X [m]')
        ax[0][0].set_ylabel('Y [m]')
        ax[0][0].grid()
        ax[0][0].axis('equal')

        # Velocity plot
        ax[1][0].set_title('Vehicle Velocity')
        ax[1][0].plot(T, all_outputs["velocity_target0"], color='tab:blue', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_target1"], color='tab:orange', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_target2"], color='tab:green', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_tracker0"], color='magenta', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_tracker1"], color='red', linestyle='-')
        ax[1][0].plot(T, all_outputs["velocity_circle"], color='tab:red', linestyle='-')
        # ax[1][0].plot(T[:i], mpf_tracker0["velocity"][:i], color='magenta', linestyle='-')
        # ax[1][0].plot(T[:i], mpf_tracker1["velocity"][:i], color='red', linestyle='-')
        ax[1][0].set_xlabel('time [s]')
        ax[1][0].set_ylabel('velocity [m/s]')
        ax[1][0].legend([
            'Target 0',
            'Target 1',
            'Target 2',
            'Tracker 0',
            'Tracker 1',
            'Virtual Circle'], prop={'size': legend_size})
        ax[1][0].grid()
        ax[1][0].set_xlim([0, 1250])
        #ax[0][1].set_ylim([-5, 5])

        # Cooperative Formation Control Plot
        ax[2][0].set_title('Cooperative Formation Control')
        ax[2][0].plot(T, cfc_centre, c='tab:red')
        ax[2][0].plot(T, pf_target1["s"], c='tab:orange')
        ax[2][0].scatter(cfc_tracker0["broadcasts"], np.full(len(cfc_tracker0["broadcasts"]), 1.5), c='tab:red', marker='+')
        ax[2][0].scatter(cfc_target1["broadcasts"], np.full(len(cfc_target1["broadcasts"]), 1.5), c='tab:orange', marker='+')
        ax[2][0].set_xlabel('time [s]')
        ax[2][0].set_ylabel('Coordination State $\gamma$')
        ax[2][0].legend([
            'Virtual Circle',
            'Target 1',
            'Broadcast Virtual Circle',
            'Broadcast Target'], prop={'size': legend_size})
        ax[2][0].grid()
        ax[2][0].set_xlim([0, 1250])

        # # ETC Broadcasting plot
        # ax[1][1].set_title('ETC Broadcasting')
        # ax[1][1].scatter(cpf_target0["broadcasts"], np.full(len(cpf_target0["broadcasts"]), 0), c='tab:blue', marker='+')
        # ax[1][1].scatter(cpf_target1["broadcasts"], np.full(len(cpf_target1["broadcasts"]), 1), c='tab:orange', marker='+')
        # ax[1][1].scatter(cpf_target2["broadcasts"], np.full(len(cpf_target2["broadcasts"]), 2), c='tab:green', marker='+')
        # ax[1][1].scatter(cpf_tracker0["broadcasts"], np.full(len(cpf_tracker0["broadcasts"]), 3), c='magenta', marker='+')
        # ax[1][1].scatter(cpf_tracker1["broadcasts"], np.full(len(cpf_tracker1["broadcasts"]), 4), c='red', marker='+')
        # ax[1][1].set_xlim([0, T[-1]])
        # ax[1][1].set_xlabel('time [s]')
        # ax[1][1].set_ylabel('Broadcasts')
        # ax[1][1].legend([
        #     'Target 0',
        #     'Target 1',
        #     'Target 2',
        #     'Tracker 0',
        #     'Tracker 1'], prop={'size': legend_size})
        # ax[1][1].grid()
        # ax[1][1].set_xlim([0, 1250])

        # Range-measurement plot
        measurements0 = [[[], []], [[], []], [[], []]]
        measurements1 = [[[], []], [[], []], [[], []]]
        for j in range(len(T)):
            if all_outputs["range00"][j] != None:
                measurements0[0][0].append(T[j])
                measurements0[0][1].append(all_outputs["range00"][j])
            if all_outputs["range01"][j] != None:
                measurements1[0][0].append(T[j])
                measurements1[0][1].append(all_outputs["range01"][j])
            if all_outputs["range10"][j] != None:
                measurements0[1][0].append(T[j])
                measurements0[1][1].append(all_outputs["range10"][j])
            if all_outputs["range11"][j] != None:
                measurements1[1][0].append(T[j])
                measurements1[1][1].append(all_outputs["range11"][j])
            if all_outputs["range20"][j] != None:
                measurements0[2][0].append(T[j])
                measurements0[2][1].append(all_outputs["range20"][j])
            if all_outputs["range21"][j] != None:
                measurements1[2][0].append(T[j])
                measurements1[2][1].append(all_outputs["range21"][j])
        ax[0][1].plot(measurements0[0][0], measurements0[0][1], color='tab:blue', linestyle='-')
        ax[0][1].plot(measurements1[0][0], measurements1[0][1], color='tab:blue', linestyle='-', label='_nolegend_')
        ax[0][1].plot(measurements0[1][0], measurements0[1][1], color='tab:orange', linestyle='-')
        ax[0][1].plot(measurements1[1][0], measurements1[1][1], color='tab:orange', linestyle='-', label='_nolegend_')
        ax[0][1].plot(measurements0[2][0], measurements0[2][1], color='tab:green', linestyle='-')
        ax[0][1].plot(measurements1[2][0], measurements1[2][1], color='tab:green', linestyle='-', label='_nolegend_')
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].set_ylabel('distance measure [m]')
        ax[0][1].legend(['Target 0', 'Target 1', 'Target 2'])
        ax[0][1].set_title('Range-measurements')
        ax[0][1].grid()

        # # Vehicle path progression plot
        # ax[2][0].set_title('Vehicle Path Progression')
        # ax[2][0].plot(T, pf_target0["s"], c='tab:blue')
        # ax[2][0].plot(T, pf_target1["s"], c='tab:orange')
        # ax[2][0].plot(T, pf_target2["s"], c='tab:green')
        # ax[2][0].plot(T, pf_tracker0["s"], c='magenta')
        # ax[2][0].plot(T, pf_tracker1["s"], c='red')
        # ax[2][0].set_xlabel('time [s]')
        # ax[2][0].set_ylabel('Coordination State $\gamma$')
        # ax[2][0].legend([
        #     'Target 0',
        #     'Target 1',
        #     'Target 2',
        #     'Tracker 0',
        #     'Tracker 1'], prop={'size': legend_size})
        # ax[2][0].grid()
        # ax[2][0].set_xlim([0, 1250])

        # EKF Velocity plot
        error = [[], [], []]
        for j in range(len(T)):
            error[0].append(np.sqrt(np.power(ekf_target0["x"][j] - all_outputs["x_target0"][j], 2) + np.power(ekf_target0["y"][j] - all_outputs["y_target0"][j], 2)))
            error[1].append(np.sqrt(np.power(ekf_target1["x"][j] - all_outputs["x_target1"][j], 2) + np.power(ekf_target1["y"][j] - all_outputs["y_target1"][j], 2)))
            error[2].append(np.sqrt(np.power(ekf_target2["x"][j] - all_outputs["x_target2"][j], 2) + np.power(ekf_target2["y"][j] - all_outputs["y_target2"][j], 2)))
        ax[1][1].plot(T, error[0], linestyle='-', color='tab:blue')
        ax[1][1].plot(T, error[1], linestyle='-', color='tab:orange')
        ax[1][1].plot(T, error[2], linestyle='-', color='tab:green')
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('distance [m]')
        #ax[1][1].legend(['x_dot', 'y_dot', 'velocity'])
        ax[1][1].set_title('EKF Error')
        ax[1][1].legend(['Target 0', 'Target 1', 'Target 2'])
        ax[1][1].grid()
        
        # # Lapierre output u plot
        # ax[0][1].set_title('Lapierre Output')
        # ax[0][1].plot(T, all_outputs["u_target0"])
        # ax[0][1].plot(T, all_outputs["u_target1"])
        # ax[0][1].plot(T, all_outputs["u_target2"])
        # ax[0][1].plot(T, all_outputs["u_tracker0"], color='magenta', linestyle='-')
        # ax[0][1].plot(T, all_outputs["u_tracker1"], color='red', linestyle='-')
        # ax[0][1].set_xlabel('time [s]')
        # ax[0][1].set_ylabel('angle rate [rad/s]')
        # ax[0][1].set_ylim([-5, 5])
        # ax[0][1].grid()
        # ax[0][1].set_xlim([0, 1250])
        # ax[0][1].legend([
        #     'Target 0',
        #     'Target 1',
        #     'Target 2',
        #     'Tracker 0',
        #     'Tracker 1'], prop={'size': legend_size})

        # Target position and prediciton plot
        position_error = [[] , [], []]
        for j in range(len(T)):
            position_error[0].append(np.linalg.norm(np.array([all_outputs["x_target0"][j] - all_outputs["x_pred_target0"][j], all_outputs["y_target0"][j] - all_outputs["y_pred_target0"][j]]))) #np.sqrt(np.power(all_outputs["x_target0"][j] - all_outputs["x_pred_target0"][j], 2) + np.power(all_outputs["y_target0"][j] - all_outputs["y_pred_target0"][j], 2)))
            position_error[1].append(np.linalg.norm(np.array([all_outputs["x_target1"][j] - all_outputs["x_pred_target1"][j], all_outputs["y_target1"][j] - all_outputs["y_pred_target1"][j]])))
            position_error[2].append(np.linalg.norm(np.array([all_outputs["x_target2"][j] - all_outputs["x_pred_target2"][j], all_outputs["y_target2"][j] - all_outputs["y_pred_target2"][j]])))
        ax[2][1].set_title('CKF Error')
        ax[2][1].plot(T, position_error[0], linestyle='-', color='tab:blue')
        ax[2][1].plot(T, position_error[1], linestyle='-', color='tab:orange')
        ax[2][1].plot(T, position_error[2], linestyle='-', color='tab:green')
        ax[2][1].set_xlabel('time [s]')
        ax[2][1].set_ylabel('distance [m]')
        ax[2][1].grid()
        ax[2][1].legend(['Target 0', 'Target 1', 'Target 2'])
        

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 

    
    if Movie == True:
        for i in range(len(T)):
            if i % frame_rate == 0 or i == len(T):

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

                ax[0][0].plot(all_outputs["x_target0"][:i], all_outputs["y_target0"][:i], color='tab:blue', linestyle='--', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_target1"][:i], all_outputs["y_target1"][:i], color='tab:orange', linestyle='--', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_target2"][:i], all_outputs["y_target2"][:i], color='tab:green', linestyle='--', label='_nolegend_')

                ax[0][0].plot(all_outputs["x_tracker0"][:i], all_outputs["y_tracker0"][:i], color='magenta', linestyle='--', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_tracker1"][:i], all_outputs["y_tracker1"][:i], color='red', linestyle='--', label='_nolegend_')

                ax[0][0].plot(all_outputs["x_ekf0"][i], all_outputs["y_ekf0"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_ekf1"][i], all_outputs["y_ekf1"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf1"][i] / (2*pi) - 90), markersize=10, label='_nolegend_')
                ax[0][0].plot(all_outputs["x_ekf2"][i], all_outputs["y_ekf2"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf2"][i] / (2*pi) - 90), markersize=10, label='_nolegend_')


                p_r = pg.Path()
                circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[i]), p_target1.get_y(cfc_centre[i])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                ax[0][0].plot(all_outputs["x_target0"][i], all_outputs["y_target0"][i], color='tab:blue', marker='o')
                ax[0][0].plot(all_outputs["x_target1"][i], all_outputs["y_target1"][i], color='tab:orange', marker='o')
                ax[0][0].plot(all_outputs["x_target2"][i], all_outputs["y_target2"][i], color='tab:green', marker='o')

                ax[0][0].plot(all_outputs["x_pred_target0"][i], all_outputs["y_pred_target0"][i], color='tab:gray', marker='o')
                ax[0][0].plot(all_outputs["x_pred_target1"][i], all_outputs["y_pred_target1"][i], color='tab:gray', marker='o', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_pred_target2"][i], all_outputs["y_pred_target2"][i], color='tab:gray', marker='o', label='_nolegend_')


                ax[0][0].plot(all_outputs["x_tracker0"][i], all_outputs["y_tracker0"][i], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_tracker1"][i], all_outputs["y_tracker1"][i], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][i] / (2*pi) - 90), markersize=10)
                
                
                
                ax[0][0].legend([
                    'Target Path 0',
                    'Target Path 1',
                    'Target Path 2',
                    'target ekf',
                    'virtual circle',
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'target pred',
                    'Tracker 0',
                    'Tracker 1'
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
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'Tracker 0',
                    'Tracker 1'], prop={'size': 6})
                ax[1][0].grid()

                # Cooperative Formation Control Plot
                ax[0][1].set_title('Cooperative Formation Control', y=1.0, pad=-14)
                ax[0][1].plot(T[:i], cfc_centre[:i], c='tab:red')
                ax[0][1].plot(T[:i], pf_target1["s"][:i], c='tab:orange')
                broadcasts = [[], []]
                for j in range(len(cfc_tracker0["broadcasts"])):
                    if cfc_tracker0["broadcasts"][j] <= T[i]:
                        broadcasts[0].append(cfc_tracker0["broadcasts"][j])
                for j in range(len(cfc_target1["broadcasts"])):
                    if cfc_target1["broadcasts"][j] <= T[i]:
                        broadcasts[1].append(cfc_target1["broadcasts"][j])
                ax[0][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), -1), c='tab:red', marker='+')
                ax[0][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), -1), c='tab:orange', marker='+')
                ax[0][1].set_xlabel('time [s]')
                ax[0][1].set_ylabel('gammas')
                ax[0][1].legend([
                    'Moving Path',
                    'Target 1',
                    'Broadcast Moving Path',
                    'Broadcast Target'], prop={'size': 6})
                ax[0][1].grid()

                # # ETC Broadcasting plot
                # ax[1][1].set_title('ETC Broadcasting', y=1.0, pad=-14)
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
                # for j in range(len(cpf_tracker0["broadcasts"])):
                #     if cpf_tracker0["broadcasts"][j] <= T[i]:
                #         broadcasts[3].append(cpf_tracker0["broadcasts"][j])
                # for j in range(len(cpf_tracker1["broadcasts"])):
                #     if cpf_tracker1["broadcasts"][j] <= T[i]:
                #         broadcasts[4].append(cpf_tracker1["broadcasts"][j])

                # ax[1][1].scatter(broadcasts[0], np.full(len(broadcasts[0]), 0), c='tab:blue', marker='+')
                # ax[1][1].scatter(broadcasts[1], np.full(len(broadcasts[1]), 1), c='tab:orange', marker='+')
                # ax[1][1].scatter(broadcasts[2], np.full(len(broadcasts[2]), 2), c='tab:green', marker='+')
                # ax[1][1].scatter(broadcasts[3], np.full(len(broadcasts[3]), 3), c='magenta', marker='+')
                # ax[1][1].scatter(broadcasts[4], np.full(len(broadcasts[4]), 4), c='red', marker='+')
                # if T[i] != 0:
                #     ax[1][1].set_xlim([0, T[i]])
                # ax[1][1].set_xlabel('time [s]')
                # ax[1][1].set_ylabel('Broadcasts')
                # ax[1][1].legend([
                #     'Target 0',
                #     'Target 1',
                #     'Target 2',
                #     'Tracker 0',
                #     'Tracker 1'], prop={'size': 6})
                # ax[1][1].grid()

                # Range Measurement
                measurements0 = [[[], []], [[], []], [[], []]]
                measurements1 = [[[], []], [[], []], [[], []]]
                for j in range(i):
                    if all_outputs["range00"][j] != None:
                        measurements0[0][0].append(T[j])
                        measurements0[0][1].append(all_outputs["range00"][j])
                    if all_outputs["range01"][j] != None:
                        measurements1[0][0].append(T[j])
                        measurements1[0][1].append(all_outputs["range01"][j])
                    if all_outputs["range10"][j] != None:
                        measurements0[1][0].append(T[j])
                        measurements0[1][1].append(all_outputs["range10"][j])
                    if all_outputs["range11"][j] != None:
                        measurements1[1][0].append(T[j])
                        measurements1[1][1].append(all_outputs["range11"][j])
                    if all_outputs["range20"][j] != None:
                        measurements0[2][0].append(T[j])
                        measurements0[2][1].append(all_outputs["range20"][j])
                    if all_outputs["range21"][j] != None:
                        measurements1[2][0].append(T[j])
                        measurements1[2][1].append(all_outputs["range21"][j])
                ax[1][1].plot(measurements0[0][0][:i], measurements0[0][1][:i], color='tab:blue', linestyle='-')
                ax[1][1].plot(measurements1[0][0][:i], measurements1[0][1][:i], color='tab:blue', linestyle='-', label='_nolegend_')
                ax[1][1].plot(measurements0[1][0][:i], measurements0[1][1][:i], color='tab:orange', linestyle='-')
                ax[1][1].plot(measurements1[1][0][:i], measurements1[1][1][:i], color='tab:orange', linestyle='-', label='_nolegend_')
                ax[1][1].plot(measurements0[2][0][:i], measurements0[2][1][:i], color='tab:green', linestyle='-')
                ax[1][1].plot(measurements1[2][0][:i], measurements1[2][1][:i], color='tab:green', linestyle='-', label='_nolegend_')
                ax[1][1].set_xlabel('time [s]')
                ax[1][1].set_ylabel('distance measure [m]')
                ax[1][1].legend(['target0', 'target1', 'target2'])
                ax[1][1].set_title('Range Measurements')
                ax[1][1].grid()

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
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'Tracker 0',
                    'Tracker 1'], prop={'size': 6})
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
                ax[1][2].set_ylim([-5, 5])
                ax[1][2].grid()
                ax[1][2].legend([
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'Tracker 0',
                    'Tracker 1'], prop={'size': 6})


                fig.show()
                plt.pause(0.001)
        
        
        input("Press Enter to end plotting...")        
        plt.pause(0.1)