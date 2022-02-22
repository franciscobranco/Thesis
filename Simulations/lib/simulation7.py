"""

Author: Francisco Branco
Created: 17/02/2022
Description: Target Pursuit Simple example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.systembuild as sb


def run_simulation():
    # Path parameters
    resolution = 40

    start_target = 0.0
    position_target = np.array([0.0, 0.0])
    orientation_target = -pi/2#0.0
    size_target = 15.0
    arc_target = 2*pi
    radius_target = size_target

    start_tracker = 0.0
    position_tracker = np.array([0.0, 0.0])
    orientation_tracker = -pi/2
    size_tracker = 5.0
    arc_tracker = 2*pi
    radius_tracker = size_tracker

    # Path creation
    p_target = pg.Path()
    circle_target = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target.append_path(circle_target)
    #p_target = pg.Path()
    #line_target = pg.Line(resolution, position_target, orientation_target, size_target, start_target)
    #p_target.append_path(line_target)

    p_tracker = pg.Path()
    circle_tracker = pg.Circle(resolution, position_tracker, orientation_tracker, arc_tracker, radius_tracker, start_tracker)
    p_tracker.append_path(circle_tracker)


    # Time parameters
    total_time = 350
    num_points = 7000
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10.0
    y = -10.0
    theta_m = 0.0
    s = 0.0

    pf_params = {
        "gamma": 1.0,
        "k1": 1.0,
        "k2": 0.3,
        "k_delta": 1.0,
        "theta_a": 0.8
    }

    # EKF parameters
    F_matrix = np.array([[1, 0, dt, 0],
                         [0, 1, 0, dt],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    
    Q_matrix = 10 * np.exp(-6) * np.array([[10, 0, 0, 0],
                              [0, 10, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])

    R_matrix = 0.01

    ekf_params = {
        "F_matrix": F_matrix,
        "Q_matrix": Q_matrix,
        "R_matrix": R_matrix
    }

    # Amount of time in seconds the target is not moving at the beginning
    time_halted = 25


    # System creation along with initial conditions
    auv_system = sb.ASVMPFOnAUVTargetPursuit(
        p_target,
        p_tracker,
        pf_params=pf_params,
        ekf_params=ekf_params,
        time_halted=time_halted,
        history=True,
        dt=dt
    )

    ic = {
        "x_target": -4.0,
        "y_target": -10.0,
        "theta_m_target": theta_m,
        "s_target": s,
        "x_follower": 0.0,
        "y_follower": -12.0,
        "theta_m_follower": theta_m,
        "s_follower": s
    }
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)
        #input()

    # Get past values for plotting
    all_outputs, _, pf_target, _, pf_tracker, ekf_tracker = auv_system.past_values()
    
    # Start plotting
    fig, ax1 = plt.subplots(2,3)
    plt.ion()
    #fig.set_size_inches((7, 14))
    manager = plt.get_current_fig_manager()
    manager.full_screen_toggle()

    frame_rate = num_points / total_time
    
    """
    
    p0.plot_path(ax1[0][0])
    

    ax1[0][0].plot(all_outputs["x0"], all_outputs["y0"], 'r--')
    ax1[0][0].plot(all_outputs["x1"], all_outputs["y1"], 'm--')

    # Labels and grid
    ax1[0][0].set_title('AUV position plot')
    ax1[0][0].set_xlabel('X [m]')
    ax1[0][0].set_ylabel('Y [m]')
    ax1[0][0].grid()
    ax1[0][0].legend(['target\'s path', 'target', 'follower'])

    # Broadcast plot
    ax1[1][1].set_title('AUV Broadcasting plot')
    ax1[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
    ax1[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
    ax1[1][1].set_xlabel('time [s]')
    ax1[1][1].legend(['target', 'follower'])

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
    laps = 0
    for count in range(len(all_outputs["s0"])):
        if count != 0 and last_value > 0.9 + all_outputs["s1"][count]:
            laps = laps + 1
        difference.append(all_outputs["s0"][count] - (all_outputs["s1"][count] + laps) / factor)
        last_value = all_outputs["s1"][count]
    ax1[0][1].plot(T, difference)
    
    #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
    ax1[0][1].set_xlabel('time [s]')
    ax1[0][1].grid()


    
    # s1 y1 plot
    ax1[0][2].set_title('AUV Lapierre s1 and y1')
    ax1[0][2].plot(T, pf0["y1_geo"])
    ax1[0][2].plot(T, pf0["s1_geo"])
    ax1[0][2].plot(T, pf1["y1_geo"])
    ax1[0][2].plot(T, pf1["s1_geo"])
    ax1[0][2].legend(['target y1', 'target s1', 'follower y1', 'follower s1'])
    

    fig.show()
    plt.pause(100)

    """

    for i in range(len(T)):
        if i % frame_rate == 0:

            if i != len(T) - 1:
                ax1[0][0].cla()
                ax1[1][0].cla()
                ax1[0][1].cla()
                ax1[1][1].cla()
                ax1[0][2].cla()
                ax1[1][2].cla()

            p_target.plot_path(ax1[0][0])

            ax1[0][0].plot(all_outputs["x_target"][i], all_outputs["y_target"][i], color='tab:blue', marker='o')

            p_r = pg.Path()
            circle_r = pg.Circle(resolution, np.array([all_outputs["x_target"][i], all_outputs["y_target"][i]]), all_outputs["theta_m_target"][i], arc_tracker, radius_tracker, start_tracker)
            p_r.append_path(circle_r)
            p_r.plot_path(ax1[0][0])

            # Plot vehicle and past course
            ax1[0][0].plot(all_outputs["x_tracker"][i], all_outputs["y_tracker"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m_tracker"][i] / (2*pi) - 90), markersize=10)
            
            
            ax1[0][0].plot(all_outputs["x_ekf"][i], all_outputs["y_ekf"][i], color='purple', marker=(3, 0, 360 * all_outputs["theta_ekf"][i] / (2*pi) - 90), markersize=10)
            ax1[0][0].plot(all_outputs["x_tracker"][:i], all_outputs["y_tracker"][:i], 'r--')
            ax1[0][0].plot(all_outputs["x_ekf"][:i], all_outputs["y_ekf"][:i], color='purple', linestyle='--')

            # Plot the virtual target
            #X0, Y0 = p_target.get_xy(all_outputs["s0"][i])
            #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
            
            
            
            #ax1[0][0].plot(X1, Y1, 'go')
            ax1[0][0].legend(['target path','target', 'moving path', 'tracker', 'estimate'], bbox_to_anchor=(0.75, 0.75))


            # Labels and grid
            ax1[0][0].set_title('Position plot')
            ax1[0][0].set_xlabel('X [m]')
            ax1[0][0].set_ylabel('Y [m]')
            ax1[0][0].grid()
            ax1[0][0].axis('equal')

            # Velocity plot
            ax1[1][0].set_title('Velocity plot')
            ax1[1][0].plot(T[:i], all_outputs["velocity_target"][:i])
            ax1[1][0].plot(T[:i], all_outputs["velocity_tracker"][:i])
            #ax1[1].set_ylim([0.98, 1.02])
            ax1[1][0].set_xlabel('time [s]')
            ax1[1][0].set_ylabel('Velocity [m/s]')
            ax1[1][0].legend(['target', 'tracker'])
            ax1[1][0].grid()

            """
            # Error plot
            ax1[0][1].set_title('Gamma Error plot for Trackers')
            difference = []
            
            for count in range(i):
                if all_outputs["s_follower0"][count] >= 0 and all_outputs["s_follower0"][count] <= 0.25 and all_outputs["s_follower1"][count] >= 0.75 and all_outputs["s_follower1"][count] <= 1:
                    difference.append(all_outputs["s_follower0"][count] + 1 - all_outputs["s_follower1"][count])
                elif all_outputs["s_follower1"][count] >= 0 and all_outputs["s_follower1"][count] <= 0.25 and all_outputs["s_follower0"][count] >= 0.75 and all_outputs["s_follower0"][count] <= 1:
                    difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count] - 1)
                else:
                    difference.append(all_outputs["s_follower0"][count] - all_outputs["s_follower1"][count])
                    #print(all_outputs["s0"][count] - all_outputs["s1"][count])
            
            ax1[0][1].plot(T[:i], difference)
            ax1[0][1].plot(T[:i], all_outputs["s_follower0"][:i])
            ax1[0][1].plot(T[:i], all_outputs["s_follower1"][:i])
            #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
            ax1[0][1].set_xlabel('time [s]')
            ax1[0][1].legend(['difference', 'follower0 s', 'follower1 s'])
            ax1[0][1].grid()
            """
            measurements = [[], []]
            # Range-measurement plot
            for j in range(i):
                if all_outputs["range"][j] >= 0:
                    measurements[0].append(T[j])
                    measurements[1].append(all_outputs["range"][j])
            ax1[0][1].plot(measurements[0], measurements[1])
            ax1[0][1].set_xlabel('time [s]')
            ax1[0][1].set_ylabel('distance measure [m]')
            #ax1[0][1].legend(['difference', 'follower0 s', 'follower1 s'])
            #ax1[0][1].title('Range-measurement Plot')
            ax1[0][1].grid()

            # EKF Velocity plot
            #ax1[1][1].plot(T[:i], ekf_tracker["x_dot"][:i])
            #ax1[1][1].plot(T[:i], ekf_tracker["y_dot"][:i])
            #ax1[1][1].plot(T[:i], all_outputs["velocity_ekf"][:i])
            #ax1[1][1].set_xlabel('time [s]')
            #ax1[1][1].set_ylabel('velocity [m/s]')
            #ax1[1][1].legend(['x_dot', 'y_dot', 'velocity'])
            #ax1[1][1].title('EKF Velocity plot')
            #ax1[1][1].grid()
            error = []
            for j in range(i):
                error.append(np.sqrt(np.power(ekf_tracker["x"][j] - all_outputs["x_target"][j], 2) + np.power(ekf_tracker["y"][j] - all_outputs["y_target"][j], 2)))

            ax1[1][1].plot(T[:i], error[:i])
            ax1[1][1].set_xlabel('time [s]')
            ax1[1][1].set_ylabel('distance [m]')
            #ax1[1][1].legend(['x_dot', 'y_dot', 'velocity'])
            ax1[1][1].set_title('EKF Error plot')
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
            ax1[0][2].set_title('Lapierre s1 and y1')
            ax1[0][2].plot(T[:i], pf_target["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf_target["s1_geo"][:i])
            ax1[0][2].plot(T[:i], pf_tracker["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf_tracker["s1_geo"][:i])
            ax1[0][2].legend(['target y1', 'target s1', 'tracker y1', 'tracker s1'])
            
            # Lapierre output u plot
            ax1[1][2].set_title('Lapierre output u')
            ax1[1][2].plot(T[:i], all_outputs["u_target"][:i])
            ax1[1][2].plot(T[:i], all_outputs["u_tracker"][:i])
            ax1[1][2].legend(['target', 'tracker'])


            

            fig.show()
            plt.pause(0.001)
            
    plt.pause(100)