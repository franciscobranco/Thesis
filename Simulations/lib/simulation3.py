"""

Author: Francisco Branco
Created: 10/09/2021
Description: ETC Cooperative Path Following example

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
    position1 = np.array([0, 0])
    position2 = np.array([0, 0])
    orientation = -pi/2
    size = 15.0
    arc = 2*pi
    radius = size

    # Path creation
    p0 = pg.Path()
    circle0 = pg.Circle(resolution, position1, orientation, arc, radius, start)
    p0.append_path(circle0)

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position2, orientation, arc, radius - 5, start)
    p1.append_path(circle1)

    # Time parameters
    T, dt = np.linspace(start=0, stop=100, num=2000, retstep=True)
    
    # Vehicle initial conditions
    x = 0
    y = 0
    theta_m = 0
    s = 0

    # ETC Parameters
    params = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "norm0": p0.total_distance,
        "norm1": p1.total_distance,
        "speed_profile0": 1,
        "speed_profile1": 2/3
    }

    # System creation along with initial conditions
    auv_pf_system = sb.DoubleAUVCPFETC(p0, p1, k_csi=0.1, gamma=1, k1=1, k2=0.3, k_delta=1, theta_a=0.8, speed_profile=1, params=params, etc_type="Time", history=True, dt=dt)
    ic = {"x0": x, "y0": y, "theta_m0": theta_m, "s0": s, "x1": x, "y1": y, "theta_m1": theta_m, "s1": s}
    auv_pf_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_pf_system.update(t)

    # Get past values for plotting
    all_outputs, _, pf0, cpf0, _, pf1, cpf1 = auv_pf_system.past_values()
    
    # Start plotting
    fig, ax1 = plt.subplots(2,3)
    plt.ion()
    fig.set_size_inches((7, 14))

    
    p0.plot_path(ax1[0][0])
    p1.plot_path(ax1[0][0])

    ax1[0][0].plot(all_outputs["x0"], all_outputs["y0"], 'r--')
    ax1[0][0].plot(all_outputs["x1"], all_outputs["y1"], 'm--')

    # Labels and grid
    ax1[0][0].set_title('AUV position plot')
    ax1[0][0].set_xlabel('X [m]')
    ax1[0][0].set_ylabel('Y [m]')
    ax1[0][0].grid()
    ax1[0][0].legend(['target\'s path', 'target', 'follower'])

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
    for count in range(len(all_outputs["s0"])):
        difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
    ax1[0][1].plot(T, difference)
    
    #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
    ax1[0][1].set_xlabel('time [s]')
    ax1[0][1].grid()

    # Broadcast plot
    ax1[1][1].set_title('AUV Broadcasting plot')
    ax1[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
    ax1[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
    ax1[1][1].set_xlabel('time [s]')
    ax1[1][1].legend(['target', 'follower'])

    
    # s1 y1 plot
    ax1[0][2].set_title('AUV s1 and y1')
    ax1[0][2].plot(T, pf0["y1_geo"])
    ax1[0][2].plot(T, pf0["s1_geo"])
    ax1[0][2].plot(T, pf1["y1_geo"])
    ax1[0][2].plot(T, pf1["s1_geo"])
    ax1[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])
    

    fig.show()
    plt.pause(100)


    """
    i = 0

    for i in range(len(T)):
        p0.plot_path(ax1[0][0])
        p1.plot_path(ax1[0][0])

        

        # Plot vehicle and past course
        ax1[0][0].plot(all_outputs["x0"][i], all_outputs["y0"][i], color='r', marker=(3, 0, 360 * all_outputs["theta_m0"][i] / (2*pi) - 90), markersize=10)
        ax1[0][0].plot(all_outputs["x1"][i], all_outputs["y1"][i], color='m', marker=(3, 0, 360 * all_outputs["theta_m1"][i] / (2*pi) - 90), markersize=10)

        ax1[0][0].plot(all_outputs["x0"][:i], all_outputs["y0"][:i], 'r--')
        ax1[0][0].plot(all_outputs["x1"][:i], all_outputs["y1"][:i], 'm--')

        # Plot the virtual target
        X0, Y0 = p0.get_xy(all_outputs["s0"][i])
        X1, Y1 = p1.get_xy(all_outputs["s1"][i])
        ax1[0][0].plot(X0, Y0, 'go')
        ax1[0][0].plot(X1, Y1, 'go')


        # Labels and grid
        ax1[0][0].set_title('AUV position plot')
        ax1[0][0].set_xlabel('X [m]')
        ax1[0][0].set_ylabel('Y [m]')
        ax1[0][0].grid()
        ax1[0][0].legend(['vehicle0 path', 'vehicle1 path', 'vehicle0', 'vehicle1'])

        # Velocity plot
        ax1[1][0].set_title('AUV Velocity plot')
        ax1[1][0].plot(T[:i], all_outputs["velocity0"][:i])
        ax1[1][0].plot(T[:i], all_outputs["velocity1"][:i])
        #ax1[1].set_ylim([0.98, 1.02])
        ax1[1][0].set_xlabel('time [s]')
        ax1[1][0].set_ylabel('Velocity [m/s]')
        ax1[1][0].grid()
        ax1[1][0].legend(['target', 'follower'])

        # Error plot
        ax1[0][1].set_title('AUV Gamma Error plot')
        difference = []
        for count in range(i):
            difference.append(all_outputs["s0"][count] - all_outputs["s1"][count])
        ax1[0][1].plot(T[:i], difference)
        #ax1[0][1].plot(T[:i], all_outputs["s1"][:i])
        ax1[0][1].set_xlabel('time [s]')
        ax1[0][1].grid()

        # Broadcast plot
        ax1[1][1].set_title('AUV Broadcasting plot')
        ax1[1][1].scatter(cpf0["broadcasts"], np.full(len(cpf0["broadcasts"]), 0), c='blue', marker='+')
        ax1[1][1].scatter(cpf1["broadcasts"], np.full(len(cpf1["broadcasts"]), 1), c='orange', marker='+')
        ax1[1][1].set_xlabel('time [s]')
        ax1[1][1].legend(['target', 'follower'])

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
        ax1[0][2].set_title('AUV s1 and y1')
        ax1[0][2].plot(T[:i], pf0["y1_geo"][:i])
        ax1[0][2].plot(T[:i], pf0["s1_geo"][:i])
        ax1[0][2].plot(T[:i], pf1["y1_geo"][:i])
        ax1[0][2].plot(T[:i], pf1["s1_geo"][:i])
        ax1[0][2].legend(['vehicle0 y1', 'vehicle0 s1', 'vehicle1 y1', 'vehicle1 s1'])


        fig.show()
        plt.pause(0.001)

        if i != len(T) - 1:
            ax1[0][0].cla()
            ax1[1][0].cla()
            ax1[0][1].cla()
            ax1[1][1].cla()
            ax1[0][2].cla()
        else:
            plt.pause(100)

    """