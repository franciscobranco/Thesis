"""

Author: Francisco Branco
Created: 15/01/2021

"""

import utils
import numpy as np
import sys
from time import sleep
from math import pi, sin, cos, tan
import matplotlib.pyplot as plt
from abc import abstractmethod


class Path:
    def __init__(self):
        self.path_list = []
        self.total_distance = 0.0
    
    def append_path(self, new_path):
        self.path_list.append(new_path)
        self.total_distance += new_path.size

    def get_section_from_s(self, s):
        i = 0
        past_distance = 0

        if s > 1:
            s = 1
        if s < 0:
            s = 0

        while s * self.total_distance > self.path_list[i].size:
            if i != 0:
                past_distance += self.path_list[i - 1].size
            i += 1
            if i == len(self.path_list):
                break
            
        return (s - past_distance / self.total_distance) / (self.path_list[i].size / self.total_distance), self.path_list[i - 1]

    def get_xy(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_xy(inner_s)

    def get_x(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_x(inner_s)
    
    def get_y(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_y(inner_s)

    def get_theta_c(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_theta_c(inner_s)

    def get_tangent(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_tangent(inner_s)

    def get_curvature(self, s):
        inner_s, path_section = self.get_section_from_s(s)
        return path_section.get_curvature(inner_s)

    def plot_path(self, ax):
        x_array = []
        y_array = []
        for path_section in self.path_list:
            new_x_array, new_y_array = path_section.xy2plot()
            x_array = np.concatenate((x_array, new_x_array))
            y_array = np.concatenate((y_array, new_y_array))
        
        ax.plot(x_array, y_array)

    



class PathSection:
    def __init__(self, resolution=100, position=np.array([0.0, 0.0]), orientation=0, size=1.0, start=0, ptype=0,):
        self.ptype = ptype
        self.resolution = resolution
        self.position = position
        self.orientation = orientation
        self.rot_mat = utils.rotation_matrix(orientation)
        self.start = start
        self.size = size


        """
        if ptype == 0:
            self.line(size)
        elif ptype == 1:
            self.ellipse(a, b, arc)
        elif ptype == 2:
            self.lawn_mower(n_laps, size)
        else:
            sys.exit("Error on type of path; you must choose either 0 for line, 1 for ellipse or 2 for lawn-mower")
        """
    """
    def point_setup(self, dt):
        # setup side needed (it is required from previous side used - comes from Path object)
        if self.ptype not in range(0,3):
            print("Path does not need to be setup")
            return

        self.tangent = np.zeros((2, self.resolution))

        for i in range(self.resolution):
            self.X[i], self.Y[i] = self.rot_mat.dot(np.array([[self.X[i]], [self.Y[i]]])) + self.position
            if i > 0:
                self.tangent[0, i - 1] = self.X[i] - self.X[i - 1]
                self.tangent[1, i - 1] = self.Y[i] - self.Y[i - 1]
                self.tangent[:, i - 1] = self.tangent[:, i - 1] / np.linalg.norm(self.tangent[:, i - 1])
                if i == self.resolution - 1:
                    self.tangent[:, i] = self.tangent[:, i - 1]
                    #self.tangent[1, i] = self.tangent[1, i - 1]
                    self.tangent[:, i] = self.tangent[:, i] / np.linalg.norm(self.tangent[:, i])
        
        self.theta_c = np.arctan2(self.tangent[0,:], self.tangent[1,:]).flatten()

        self.Cc = utils.curvature(self.X, self.Y, dt)

        v0 = utils.rotation_matrix(self.side).dot(self.tangent[:,0])

        self.ptf = utils.ptf(self.tangent, v0)

    def test(self):
        if self.ptype not in range(0,3):
            print("Path does not need to be tested")
            return
        
        plt.figure()
        plt.ion()
        plt.suptitle('Path Testing')
        plt.xlabel('X')
        plt.ylabel('Y')

        for i in range(len(self.X)):
            plt.plot(self.X, self.Y, 'o')
            if i == 0:
                plt.show()
                plt.pause(2.5)
                plt.cla()
            
            plt.text(0, 0, "theta=" + str(self.theta_c[i]) + ", curvature=" + str(self.Cc[i]) + ", s=" + str(self.s[i]) + ", gamma=" + str(self.s_param[i]), wrap=True)
        
            plt.quiver([self.X[i], self.X[i]], [self.Y[i], self.Y[i]], [self.tangent[0, i], self.ptf[0, i]], [self.tangent[1, i], self.ptf[1, i]], color=['b','r'], scale=21)
            plt.show()
            plt.pause(0.05)
            plt.cla()
    """
    """
    @abstractmethod
    def get_xy(self, s):
        pass

    @abstractmethod
    def get_x(self, s):
        pass

    @abstractmethod
    def get_y(self, s):
        pass

    @abstractmethod
    def get_theta_c(self, s):
        pass

    @abstractmethod
    def get_tangent(self, s):
        pass

    @abstractmethod
    def get_curvature(self, s):
        pass
    """


class Line(PathSection):
    def __init__(self, resolution=100, position=np.array([0.0, 0.0]), orientation=0, size=1.0, start=0):
        super().__init__(resolution, position, orientation, size, start, ptype=0)
        
        #x_f = size + position[0]
        self.vector = utils.rotation_matrix(orientation).dot(np.array([size, 0.0])).reshape(2,)
        
        """
        self.X = np.linspace(0, size, self.resolution, endpoint=True, dtype=float)
        self.Y = np.copy(self.X)
        
        self.s = np.copy(self.X)
        self.s_param = self.s / size

        num_points = self.resolution
        t_size = size
        dt = t_size / num_points

        self.point_setup(dt)
        """

    # change this to the same as circle
    def get_xy(self, s):
        if self.start == 0:
            return s * self.vector[0] + self.position[0], s * self.vector[1] + self.position[1]
        else:
            return s * (-1) * self.vector[0] + (self.position[0] + self.vector[0]), s * (-1) * self.vector[1] + (self.position[1] + self.vector[1])

    def get_x(self, s):
        if self.start == 0:
            return s * self.vector[0] + self.position[0]
        else:
            return s * (-1) * self.vector[0] + (self.position[0] + self.vector[0])

    def get_y(self, s):
        if self.start == 0:
            return s * self.vector[1] + self.position[1]
        else:
            return s * (-1) * self.vector[1] + (self.position[1] + self.vector[1])

    def get_theta_c(self, s):
        # Careful for angle wrap
        # also careful for opposite start of path
        tangent = self.get_tangent(s)
        return np.arctan2(tangent[1], tangent[0])

    def get_tangent(self, s):
        norm = np.linalg.norm(self.vector)
        if self.start == 0:
            return self.vector / norm
        else:
            return (-1)*self.vector / norm

    def get_curvature(self, s):
        return 0
    
    def xy2plot(self):
        #this is wrong
        """
        x = np.linspace(self.position[0], self.position[0] + self.vector[0], self.resolution)
        y = np.linspace(self.position[1], self.position[1] + self.vector[1], self.resolution)
        """
        x = np.linspace(0, self.size, self.resolution)
        y = np.zeros((self.resolution,))
        position_vector = np.zeros((2, self.resolution))
        for i in range(self.resolution):
            position_vector[0][i] = self.position[0]
            position_vector[1][i] = self.position[1]
        plot_matrix = utils.rotation_matrix(self.orientation).dot(np.array([x, y])) + position_vector

        return plot_matrix[0], plot_matrix[1]



class Circle(PathSection):
    def __init__(self, resolution=100, position=np.array([0.0, 0.0]), orientation=0, arc=2*pi, radius=1.0, start=0):
        super().__init__(resolution, position, orientation, ptype=1, size=radius*arc)
        
        # Missing the arc length
        self.arc = arc
        self.radius = radius

        """
        t = np.linspace(0, 2*pi, self.resolution, endpoint=True, dtype=float)
        self.X = a * np.cos(t)
        self.Y = b * np.sin(t)

        self.s = np.copy(t)
        self.s_param = self.s / (2*pi)

        num_points = self.resolution
        t_size = 2*pi
        dt = t_size / num_points

        self.point_setup(dt)
        """

    def get_xy(self, s: float) -> float:
        if self.start == 0:
            X = self.radius * cos(self.arc*s)
            Y = self.radius * sin(self.arc*s)
        else:
            X = self.radius * cos(self.arc*(1 - s))
            Y = self.radius * sin(self.arc*(1 - s))
        
        new_position = utils.rotation_matrix(self.orientation).dot(np.array([[X], [Y]])) + self.position.reshape((2, 1))
        
        return new_position[0][0], new_position[1][0]

    def get_x(self, s: float) -> float:
        X, _ = self.get_xy(s)
        return X

    def get_y(self, s: float) -> float:
        _, Y = self.get_xy(s)
        return Y
        
    def get_theta_c(self, s) -> float:
        # Careful for angle wrap
        tangent = self.get_tangent(s)
        return np.arctan2(tangent[1][0], tangent[0][0])

    def get_tangent(self, s):
        #wrap angles around
        X, Y = self.get_xy(s)
        vector = np.array([[X], [Y]]) - self.position.reshape((2, 1))
        #vector = np.array([[X], [Y]])
        if self.start == 0:
            tangent = utils.rotation_matrix(pi/2).dot(vector)
            norm = np.linalg.norm(tangent)
            #print("tangent = " + str(tangent))
            return tangent / norm
        else:
            tangent = utils.rotation_matrix((-1)*pi/2).dot(vector)
            norm = np.linalg.norm(tangent)
            return tangent / norm

    def get_curvature(self, s: float) -> float:
        return 1/self.radius

    def xy2plot(self):
        for i in np.linspace(0, self.resolution, self.resolution):
            if i == 0:
                x = np.array([[self.radius * np.cos(i * self.arc / self.resolution)]])
                y = np.array([[self.radius * np.sin(i * self.arc / self.resolution)]])
                position_matrix = np.copy(self.position.reshape((2, 1)))
            else:
                x = np.concatenate((x, [[self.radius * np.cos(i * self.arc / self.resolution)]]), axis=1)
                y = np.concatenate((y, [[self.radius * np.sin(i * self.arc / self.resolution)]]), axis=1)
                position_matrix = np.concatenate((position_matrix, self.position.reshape((2, 1))), axis=1)
        
        plot_matrix = utils.rotation_matrix(self.orientation).dot(np.append(x.reshape(1, np.shape(x)[1]), y.reshape(1, np.shape(y)[1]), axis=0)) + position_matrix

        return plot_matrix[0], plot_matrix[1]
        


class LawnMower(PathSection):
    def __init__(self, ptype=0, resolution=100, side=0, position=np.array([0.0, 0.0]), orientation=0, size=[1.0, 1.0], a=1.0, b=1.0, arc=1.0, n_laps=0):
        super().__init__(resolution, side, position, orientation, size, ptype=2)

        # Verify size values and type
        if len(size) != 2 and not isinstance(size[0], float) and not isinstance(size[1], float):
            print("Invalid size")
            sys.exit()

        

    def get_curvature(self) -> (float):
        return 1/self.size

    def get_x(self, s):
        return self.size * cos(2*pi*s) + self.position[0]

    def get_y(self, s):
        return self.size * sin(2*pi*s) + self.position[1]




"""
class elipse(path):
    def __init__(self, ptype=0, resolution=100, side=0, position=np.array([[0.0], [0.0]]), orientation=0, size=1.0, a=1.0, b=1.0, arc=1.0, n_laps=0):
        super().__init__(ptype=-1, resolution, side, position, orientation, size)

        

        
        t = np.linspace(0, 2*pi, self.resolution, endpoint=True, dtype=float)
        self.X = a * np.cos(t)
        self.Y = b * np.sin(t)

        self.s = np.copy(t)
        self.s_param = self.s / (2*pi)

        num_points = self.resolution
        t_size = 2*pi
        dt = t_size / num_points

        self.point_setup(dt)
        
    
    def getCurv(self):
        return 1/self.size

    def getX(self, s):
        return self.size * cos(2*pi*s) + self.position[0]

    def getY(self, s):
        return self.size * sin(2*pi*s) + self.position[1]

"""