"""

Author: Francisco Branco
Created: 22/12/2020

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt

import control as ct
import libct.systembuild as sb

import pathgeneration as pg


# Control Library related tests
def run_simulation():

    resolution = 40
    start = 0
    position = np.array([10, 10])
    orientation = -pi/2
    size = 5.0
    arc = 2*pi
    radius = size

    p1 = pg.Path()
    circle1 = pg.Circle(resolution, position, orientation, arc, radius, start)
    p1.append_path(circle1)


    resolution = 40
    start = 0
    position = np.array([10, 7])
    orientation = -pi/2
    size = 5.0
    arc = 2*pi
    radius = size

    p2 = pg.Path()
    circle2 = pg.Circle(resolution, position, orientation, arc, radius, start)
    p2.append_path(circle2)

    auv_system = sb.setup_simple_lapierre_kinematics_auv_system(some_path=p1, gamma=1, k1=1, k2=1, k_delta=1, theta_a=pi/4)
    #auv_system = ts.setup_simple_cooperative_lapierre_kinematics_auv_system(path1=p1, path2=p2, gamma=1, k1=1, k2=1, k_delta=1, theta_a=pi/4, k_csi=5)
    

    t = np.arange(0, 35, 0.5)

    V = np.full(np.shape(t), 1)
    V_DOT = np.zeros(np.shape(t))

    t, all_outputs, state_output = ct.input_output_response(auv_system, t, [V, V_DOT, V, V_DOT], X0=(10, 10, 0, 0.3, 0, 0), return_x=True)
    #t, all_outputs, state_output = ct.input_output_response(auv_system, t, X0=(10, 10, 0, 0.3, 0, 0, 10, 7, 0, 0.1, 0, 0, 0.2, 0.2), return_x=True)
    
    #fig, (ax1, ax2) = plt.subplots(1, 2)
    fig, ax1 = plt.subplots()
    plt.ion()
    #fig.set_size_inches((12, 7))
    fig.set_size_inches((7, 7))
    
    s1_vector = []
    s2_vector = []

    for i in range(len(t)):
            p1.plot_path(ax1)
            #p2.plot_path(ax1)
            if i == 0:
                ax1.set_title('AUV position plot')
                ax1.set_xlabel('X [m]')
                ax1.set_ylabel('Y [m]')
                ax1.grid()

                #ax2.set_title('Synchronization Parameters')
                #ax2.set_xlabel('Time [s]')
                #ax2.set_ylabel(r'$\gamma$')
                #ax2.set_ylim((0, 1.2))
                #ax2.grid()

                fig.show()
                plt.pause(2.5)
                ax1.cla()

            ax1.plot(all_outputs[0][i], all_outputs[1][i], 'ro')
            #ax1.plot(all_outputs[3][i], all_outputs[4][i], 'bo')

            s1 = state_output[3][i]
            if s1 < 0:
                s1 = 0
            elif s1 > 1:
                s1 = 1
            X, Y = p1.get_xy(s1)
            ax1.plot(X, Y, 'go')

            #s2 = state_output[9][i]
            #if s2 < 0:
            #    s2 = 0
            #elif s2 > 1:
            #    s2 = 1
            #X, Y = p2.get_xy(s2)
            #ax1.plot(X, Y, 'ko')
 
            ax1.plot(all_outputs[0][:i], all_outputs[1][:i], 'r--')
            #ax1.plot(all_outputs[3][:i], all_outputs[4][:i], 'b--')

            s1_vector.append(s1)
            #s2_vector.append(s2)

            #ax2.plot(t[i], s1_vector[i], 'ro')
            #ax2.plot(t[:i], s1_vector[:i], 'r-')
            #ax2.plot(t[i], s2_vector[i], 'bo')
            #ax2.plot(t[:i], s2_vector[:i], 'b-')
            
            ax1.set_title('AUV position plot')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.grid()

            #ax2.set_title('Synchronization Parameters')
            #ax2.set_xlabel('Time [s]')
            #ax2.set_ylabel(r'$\gamma$')
            #ax2.set_ylim((0, 1.05))
            #ax2.grid()

            fig.show()
            plt.pause(0.05)
            if i != len(t) - 1:
                ax1.cla()
                #ax2.cla()
            else:
                plt.pause(100)