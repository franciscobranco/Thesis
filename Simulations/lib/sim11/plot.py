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
    all_outputs, kine_target0, pf_target0, cpf_target0, kine_target1, pf_target1, cpf_target1, cfc_target1, kine_target2, pf_target2, cpf_target2, mpf_tracker0, pf_tracker0, cpf_tracker0, cfc_tracker0, cfc_centre, mpf_tracker1, pf_tracker1, cpf_tracker1 = past_values

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

    # # ax.set_title('Position plot')
    # p_target0.plot_path(ax)
    # p_target1.plot_path(ax)
    # p_target2.plot_path(ax)

    # ax.plot(p_target0.get_x(pf_target0["s"][-1]), p_target0.get_y(pf_target0["s"][-1]), color='lightskyblue', marker='o', label='_nolegend_')
    # ax.plot(p_target1.get_x(pf_target1["s"][-1]), p_target1.get_y(pf_target1["s"][-1]), color='orange', marker='o', label='_nolegend_')
    # ax.plot(p_target2.get_x(pf_target2["s"][-1]), p_target2.get_y(pf_target2["s"][-1]), color='limegreen', marker='o', label='_nolegend_')
    # ax.plot(p_target1.get_x(cfc_centre[-1]) + p_tracker0.get_x(pf_tracker0["s"][-1]), p_target1.get_y(cfc_centre[-1]) + p_tracker0.get_y(pf_tracker0["s"][-1]), color='violet', marker='o', label='_nolegend_')
    # ax.plot(p_target1.get_x(cfc_centre[-1]) + p_tracker1.get_x(pf_tracker1["s"][-1]), p_target1.get_y(cfc_centre[-1]) + p_tracker1.get_y(pf_tracker1["s"][-1]), color='orangered', marker='o', label='_nolegend_')

    # ax.plot(all_outputs["x_target0"], all_outputs["y_target0"], color='tab:blue', linestyle='--', label='_nolegend_')
    # ax.plot(all_outputs["x_target1"], all_outputs["y_target1"], color='tab:orange', linestyle='--', label='_nolegend_')
    # ax.plot(all_outputs["x_target2"], all_outputs["y_target2"], color='tab:green', linestyle='--', label='_nolegend_')

    # ax.plot(all_outputs["x_tracker0"], all_outputs["y_tracker0"], color='magenta', linestyle='--', label='_nolegend_')
    # ax.plot(all_outputs["x_tracker1"], all_outputs["y_tracker1"], color='red', linestyle='--', label='_nolegend_')

    # p_r = pg.Path()
    # circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[-1]), p_target1.get_y(cfc_centre[-1])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
    # p_r.append_path(circle_r)
    # p_r.plot_path(ax)

    # ax.plot(all_outputs["x_target0"][-1], all_outputs["y_target0"][-1], color='tab:blue', marker='o')
    # ax.plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:orange', marker='o')
    # ax.plot(all_outputs["x_target2"][-1], all_outputs["y_target2"][-1], color='tab:green', marker='o')

    # ax.plot(all_outputs["x_tracker0"][-1], all_outputs["y_tracker0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][-1] / (2*pi) - 90), markersize=10)
    # ax.plot(all_outputs["x_tracker1"][-1], all_outputs["y_tracker1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][-1] / (2*pi) - 90), markersize=10)
    
    # ax.legend([
    #     'Target Path 0',
    #     'Target Path 1',
    #     'Target Path 2',
    #     'Virtual Circle',
    #     'Target 0',
    #     'Target 1',
    #     'Target 2',
    #     'Tracker 0',
    #     'Tracker 1'
    #     ], prop={'size': 18})

    # # Labels and grid
    # ax.set_xlabel('X [m]')
    # ax.set_ylabel('Y [m]')
    # ax.grid()
    # ax.axis('equal')

    # fig.show()
    # plt.pause(0.1)
    # input()

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

        p_r = pg.Path()
        circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[-1]), p_target1.get_y(cfc_centre[-1])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
        p_r.append_path(circle_r)
        p_r.plot_path(ax[0][0])

        ax[0][0].plot(all_outputs["x_target0"][-1], all_outputs["y_target0"][-1], color='tab:blue', marker='o')
        ax[0][0].plot(all_outputs["x_target1"][-1], all_outputs["y_target1"][-1], color='tab:orange', marker='o')
        ax[0][0].plot(all_outputs["x_target2"][-1], all_outputs["y_target2"][-1], color='tab:green', marker='o')

        ax[0][0].plot(all_outputs["x_tracker0"][-1], all_outputs["y_tracker0"][-1], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][-1] / (2*pi) - 90), markersize=10)
        ax[0][0].plot(all_outputs["x_tracker1"][-1], all_outputs["y_tracker1"][-1], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][-1] / (2*pi) - 90), markersize=10)
        
        ax[0][0].legend([
            'Target Path 0',
            'Target Path 1',
            'Target Path 2',
            'Virtual Circle',
            'Target 0',
            'Target 1',
            'Target 2',
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
        ax[2][1].set_title('Cooperative Formation Control')
        ax[2][1].plot(T, cfc_centre, c='tab:red')
        ax[2][1].plot(T, pf_target1["s"], c='tab:orange')
        ax[2][1].scatter(cfc_tracker0["broadcasts"], np.full(len(cfc_tracker0["broadcasts"]), 1.5), c='tab:red', marker='+')
        ax[2][1].scatter(cfc_target1["broadcasts"], np.full(len(cfc_target1["broadcasts"]), 1.5), c='tab:orange', marker='+')
        ax[2][1].set_xlabel('time [s]')
        ax[2][1].set_ylabel('Coordination State $\gamma$')
        ax[2][1].legend([
            'Virtual Circle',
            'Target 1',
            'Broadcast Virtual Circle',
            'Broadcast Target'], prop={'size': legend_size})
        ax[2][1].grid()
        ax[2][1].set_xlim([0, 1250])

        # ETC Broadcasting plot
        ax[1][1].set_title('ETC Broadcasting')
        ax[1][1].scatter(cpf_target0["broadcasts"], np.full(len(cpf_target0["broadcasts"]), 0), c='tab:blue', marker='+')
        ax[1][1].scatter(cpf_target1["broadcasts"], np.full(len(cpf_target1["broadcasts"]), 1), c='tab:orange', marker='+')
        ax[1][1].scatter(cpf_target2["broadcasts"], np.full(len(cpf_target2["broadcasts"]), 2), c='tab:green', marker='+')
        ax[1][1].scatter(cpf_tracker0["broadcasts"], np.full(len(cpf_tracker0["broadcasts"]), 3), c='magenta', marker='+')
        ax[1][1].scatter(cpf_tracker1["broadcasts"], np.full(len(cpf_tracker1["broadcasts"]), 4), c='red', marker='+')
        ax[1][1].set_xlim([0, T[-1]])
        ax[1][1].set_xlabel('time [s]')
        ax[1][1].set_ylabel('Broadcasts')
        ax[1][1].legend([
            'Target 0',
            'Target 1',
            'Target 2',
            'Tracker 0',
            'Tracker 1'], prop={'size': legend_size})
        ax[1][1].grid()
        ax[1][1].set_xlim([0, 1250])

        # Vehicle path progression plot
        ax[2][0].set_title('Vehicle Path Progression')
        ax[2][0].plot(T, pf_target0["s"], c='tab:blue')
        ax[2][0].plot(T, pf_target1["s"], c='tab:orange')
        ax[2][0].plot(T, pf_target2["s"], c='tab:green')
        ax[2][0].plot(T, pf_tracker0["s"], c='magenta')
        ax[2][0].plot(T, pf_tracker1["s"], c='red')
        ax[2][0].set_xlabel('time [s]')
        ax[2][0].set_ylabel('Coordination State $\gamma$')
        ax[2][0].legend([
            'Target 0',
            'Target 1',
            'Target 2',
            'Tracker 0',
            'Tracker 1'], prop={'size': legend_size})
        ax[2][0].grid()
        ax[2][0].set_xlim([0, 1250])
        
        # Lapierre output u plot
        ax[0][1].set_title('Vehicle PF Control Law')
        ax[0][1].plot(T, all_outputs["u_target0"])
        ax[0][1].plot(T, all_outputs["u_target1"])
        ax[0][1].plot(T, all_outputs["u_target2"])
        ax[0][1].plot(T, all_outputs["u_tracker0"], color='magenta', linestyle='-')
        ax[0][1].plot(T, all_outputs["u_tracker1"], color='red', linestyle='-')
        ax[0][1].set_xlabel('time [s]')
        ax[0][1].set_ylabel('angle rate [rad/s]')
        ax[0][1].set_ylim([-5, 5])
        ax[0][1].grid()
        ax[0][1].set_xlim([0, 1250])
        ax[0][1].legend([
            'Target 0',
            'Target 1',
            'Target 2',
            'Tracker 0',
            'Tracker 1'], prop={'size': legend_size})
        

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

                ax[0][0].plot(all_outputs["x_target0"][i], all_outputs["y_target0"][i], color='tab:blue', marker='o')
                ax[0][0].plot(all_outputs["x_target1"][i], all_outputs["y_target1"][i], color='tab:orange', marker='o')
                ax[0][0].plot(all_outputs["x_target2"][i], all_outputs["y_target2"][i], color='tab:green', marker='o')
                ax[0][0].plot(all_outputs["x_target0"][:i], all_outputs["y_target0"][:i], color='tab:blue', linestyle='--', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_target1"][:i], all_outputs["y_target1"][:i], color='tab:orange', linestyle='--', label='_nolegend_')
                ax[0][0].plot(all_outputs["x_target2"][:i], all_outputs["y_target2"][:i], color='tab:green', linestyle='--', label='_nolegend_')

                p_r = pg.Path()
                circle_r = pg.Circle(resolution, np.array([p_target1.get_x(cfc_centre[i]), p_target1.get_y(cfc_centre[i])]), 0., paths["p_tracker0"].path_list[0].arc, paths["p_tracker0"].path_list[0].radius, paths["p_tracker0"].path_list[0].start)
                p_r.append_path(circle_r)
                p_r.plot_path(ax[0][0])

                ax[0][0].plot(all_outputs["x_tracker0"][i], all_outputs["y_tracker0"][i], color='magenta', marker=(3, 0, 360 * all_outputs["theta_m_tracker0"][i] / (2*pi) - 90), markersize=10)
                ax[0][0].plot(all_outputs["x_tracker1"][i], all_outputs["y_tracker1"][i], color='red', marker=(3, 0, 360 * all_outputs["theta_m_tracker1"][i] / (2*pi) - 90), markersize=10)
                
                ax[0][0].plot(all_outputs["x_tracker0"][:i], all_outputs["y_tracker0"][:i], color='magenta', linestyle='--')
                ax[0][0].plot(all_outputs["x_tracker1"][:i], all_outputs["y_tracker1"][:i], color='red', linestyle='--')
                
                ax[0][0].legend([
                    'Target Path 0',
                    'Target Path 1',
                    'Target Path 2',
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'Moving Path',
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
                if T[i] != 0:
                    ax[1][1].set_xlim([0, T[i]])
                ax[1][1].set_xlabel('time [s]')
                ax[1][1].set_ylabel('Broadcasts')
                ax[1][1].legend([
                    'Target 0',
                    'Target 1',
                    'Target 2',
                    'Tracker 0',
                    'Tracker 1'], prop={'size': 6})
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