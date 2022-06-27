"""

Author: Francisco Branco
Created: 24/08/2021

"""


import numpy as np
from math import pi
import matplotlib.pyplot as plt
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

import pathgeneration as pg


def plot(paths, num_points, total_time, resolution, T, past_values, Movie):
    all_outputs, _, _, _ = past_values

    input("Press Enter to start plotting...")

    p1 = paths["p1"]
    
    # Start plotting
    fig, ax = plt.subplots(1,2)
    fig.set_size_inches((10, 5))
    fig.subplots_adjust(hspace=0.45, wspace=0.25)
    # plt.ion()

    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()

    # Movie = False

    # frame_factor = 0.25
    # frame_rate = num_points / total_time * frame_factor

    if Movie == None:
        plt.ion()

        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()

        p1.plot_path(ax[0])

        ax[0].plot(all_outputs["x"], all_outputs["y"], linestyle='--', color='red', label='_nolegend_')

        # X, Y = p1.get_xy(all_outputs["s"][100])
        # ax[0].plot(X, Y, 'go')
        # ax[0].plot(all_outputs["x"][100], all_outputs["y"][100], 'ro')

        # X, Y = p1.get_xy(all_outputs["s"][475])
        # ax[0].plot(X, Y, 'go')
        # ax[0].plot(all_outputs["x"][475], all_outputs["y"][475], 'ro')

        # X, Y = p1.get_xy(all_outputs["s"][600])
        # ax[0].plot(X, Y, 'go')
        # ax[0].plot(all_outputs["x"][600], all_outputs["y"][600], 'ro')

        # X, Y = p1.get_xy(all_outputs["s"][-1])
        # ax[0].plot(X, Y, 'go')
        ax[0].plot(all_outputs["x"][-1], all_outputs["y"][-1], marker='o', color='red')

           

        ax[0].set_title('Vehicle Position')
        ax[0].set_xlabel('X [m]')
        ax[0].set_ylabel('Y [m]')
        ax[0].grid()
        ax[0].legend(['Path', 'Vehicle'])

        # Plot Lapierre error
        error = []
        for j in range(len(T)):
            error.append(np.sqrt(np.power(all_outputs["x"][j] - p1.get_xy(all_outputs["s"][j])[0], 2) + np.power(all_outputs["y"][j] - p1.get_xy(all_outputs["s"][j])[1], 2)))
        ax[1].plot(T, error)
        ax[1].set_title('PF Error')
        ax[1].set_xlabel('time [s]')
        ax[1].set_ylabel('Distance between vehicle and virtual target [m]')
        ax[1].grid()

        fig.show()
        plt.pause(0.1)
        input("Press Enter to end plotting...") 

    else:
        # Calculate number of total frames to see final duration (default of 25 fps) 375 frames for 15s
        duration = 15

        def make_frame(t):
            i = int(len(T) * t / duration)
            if i >= len(T):
                i = len(T) - 1
                # Start by plotting vehicle position and trajectory
            
                # if i == 0:
                    # p1.plot_path(ax[0])
                    # ax[0].set_title('AUV position plot')
                    # ax[0].set_xlabel('X [m]')
                    # ax[0].set_ylabel('Y [m]')
                    # ax[0].grid()

                    # fig.show()
                    # plt.pause(1)
                    # ax[0].cla()
                    # ax[1].cla()
                
            ax[0].cla()
            ax[1].cla()

            p1.plot_path(ax[0])

            X, Y = p1.get_xy(all_outputs["s"][i])
            ax[0].plot(X, Y, 'go')

            ax[0].plot(all_outputs["x"][i], all_outputs["y"][i], 'ro')

            ax[0].plot(all_outputs["x"][:i], all_outputs["y"][:i], 'r--')

            ax[0].set_title('Vehicle Position')
            ax[0].set_xlabel('X [m]')
            ax[0].set_ylabel('Y [m]')
            ax[0].grid()
            ax[0].legend(['Path', 'Virtual Target', 'Vehicle'], loc='upper right')


            # Plot Lapierre error
            error = []
            for j in range(i):
                error.append(np.sqrt(np.power(all_outputs["x"][j] - p1.get_xy(all_outputs["s"][j])[0], 2) + np.power(all_outputs["y"][j] - p1.get_xy(all_outputs["s"][j])[1], 2)))
            ax[1].plot(T[:i], error)
            ax[1].set_title('PF Error')
            ax[1].set_xlabel('time [s]')
            ax[1].set_ylabel('Distance between vehicle and virtual target [m]')
            ax[1].grid()


            # fig.show()
            # plt.pause(0.01)

            return mplfig_to_npimage(fig)

        # fig.show()
        # plt.pause(0.1)

        animation = VideoClip(make_frame, duration=duration)
        animation.write_videofile(Movie + ".mp4", fps=10)

        input("Press Enter to end plotting...") 