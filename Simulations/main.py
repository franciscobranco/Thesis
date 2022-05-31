"""

Author: Francisco Branco
Created: 22/12/2020

"""


#import sys
#import utils
from os.path import exists
import pickle

# These modules need to be changed according to the simulation you wish to run
from lib.sim12 import run
from lib.sim12 import plot as plotting


if __name__ == '__main__':
    simulation_path = "lib\sim12\\"

    print("Welcome to the simulation world of Cooperative Multiple Formation of Vehicles!")
    print("Make sure not to forget about specifying which simulation you want to run and visualize in the code!\n")
    print("Current simulation path: " + simulation_path + "\n")
    answer = input("Execute new simulation?[Y/n] ")
    if answer == "Y" or answer == "y" or answer == "yes" or answer == "Yes" or answer == "":
        name = input("Name your simulation: ")
        run.simulation(name)
    else:
        name = input("Specify the name of the simulation file you wish to load: ")
        # The path "\lib\simX" must be changed according to the simulation you want to run
        file_exists = exists(simulation_path + name + ".txt")
        if file_exists:
            # The path must be changed here too
            with open(simulation_path + name + ".txt", 'rb') as f:
                paths = pickle.load(f)
                num_points = pickle.load(f)
                total_time = pickle.load(f)
                resolution = pickle.load(f)
                T = pickle.load(f)
                past_values = pickle.load(f)
                plotting.plot(paths, num_points, total_time, resolution, T, past_values)
                f.close()
        else:
            print("Specified file does not exist, ending simulation...")