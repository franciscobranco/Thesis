"""

Author: Francisco Branco
Created: 02/10/2021
Description: Moving Path Following example

"""



import numpy as np
from math import pi
import matplotlib.pyplot as plt

import pathgeneration as pg
import lib.systembuild as sb


def run_simulation():
    # Path parameters
    resolution = 40
    start = 0
    position0 = np.array([0, 0])
    position1 = np.array([0, 0])
    orientation = -pi/2
    size = 15.0
    arc = 2*pi
    radius = size

    # Path creation
    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position0, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position1, orientation, arc, radius - 10, start)
    p1.append_path(circle1)

    # Time parameters
    total_time = 350
    num_points = 7000
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10
    y = -10
    theta_m = 0
    s = 0
    factor = 10

    pf_params = {
        "gamma": 1,
        "k1": 1,
        "k2": 0.3,
        "k_delta": 1,
        "theta_a": 0.8
    }

    cpf_params = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": 0.1,
        "k_csi1": 0.2,
        "norm0": p0.total_distance,
        "norm1": p1.total_distance * 10,
        "speed_profile0": 0.25,
        "speed_profile1": p1.total_distance * 10 / p0.total_distance * 0.25
    }


    # System creation along with initial conditions
    auv_pf_system = sb.DoubleAUVMPFETCTest(p0, p1, pf_params=pf_params, cpf_params=cpf_params, etc_type="Time", factor=factor, history=True, dt=dt)
    ic = {"x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y - 5, "theta_m1": theta_m, "s1": s}
    auv_pf_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_pf_system.update(t)

    # Get past values for plotting
    all_outputs, _, pf0, cpf0, _, pf1, cpf1 = auv_pf_system.past_values()

    
    print("Broadcasts: " + str(len(cpf0["broadcasts"]) + len(cpf1["broadcasts"])))


    
    # Start plotting
    fig, ax1 = plt.subplots(2,3)
    plt.ion()
    fig.set_size_inches((7, 14))

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

    i = 0

    for i in range(len(T)):
        if i % frame_rate == 0:

            if i != len(T) - 1:
                ax1[0][0].cla()
                ax1[1][0].cla()
                ax1[0][1].cla()
                ax1[1][1].cla()
                ax1[0][2].cla()

            p0.plot_path(ax1[0][0])

            # Plot vehicle and past course
            ax1[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
            ax1[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

            ax1[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
            ax1[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')
            

            # Plot the virtual target
            X0, Y0 = p0.get_xy(all_outputs["s0"][i])
            #X1, Y1 = p1.get_xy(all_outputs["s1"][i])
            ax1[0][0].plot(X0, Y0, 'go')
            #ax1[0][0].plot(X1, Y1, 'go')
            ax1[0][0].legend(['target\'s path', 'target', 'follower'])


            # Labels and grid
            ax1[0][0].set_title('AUV position plot')
            ax1[0][0].set_xlabel('X [m]')
            ax1[0][0].set_ylabel('Y [m]')
            ax1[0][0].grid()
            ax1[0][0].axis('equal')

            # Velocity plot
            ax1[1][0].set_title('AUV Velocity plot')
            ax1[1][0].plot(T[:i], all_outputs["velocity0"][:i])
            ax1[1][0].plot(T[:i], all_outputs["velocity1"][:i])
            #ax1[1].set_ylim([0.98, 1.02])
            ax1[1][0].set_xlabel('time [s]')
            ax1[1][0].set_ylabel('Velocity [m/s]')
            ax1[1][0].grid()

            # Error plot
            ax1[0][1].set_title('AUV Gamma Error plot')
            difference = []
            laps = 0
            for count in range(i):
                if count != 0 and last_value > 0.9 + all_outputs["s1"][count]:
                    laps = laps + 1
                difference.append(all_outputs["s0"][count] - (all_outputs["s1"][count] + laps) / factor)
                last_value = all_outputs["s1"][count]

            
            ax1[0][1].plot(T[:i], difference)
            #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
            ax1[0][1].set_xlabel('time [s]')
            ax1[0][1].grid()

            # Broadcast plot
            ax1[1][1].set_title('AUV Broadcasting plot')
            ax1[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
            ax1[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
            ax1[1][1].set_xlabel('time [s]')

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
            ax1[0][2].set_title('AUV Lapierre s1 and y1')
            ax1[0][2].plot(T[:i], pf0["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf0["s1_geo"][:i])
            ax1[0][2].plot(T[:i], pf1["y1_geo"][:i])
            ax1[0][2].plot(T[:i], pf1["s1_geo"][:i])
            ax1[0][2].legend(['target y1', 'target s1', 'follower y1', 'follower s1'])
            

            fig.show()
            plt.pause(0.001)

    plt.pause(100)