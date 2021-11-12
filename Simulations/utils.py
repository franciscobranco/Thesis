"""

Author: Francisco Branco
Created: 14/01/2021

"""

import numpy as np
from math import pi, isnan, fmod

# Delta function chosen for Lapierre path following
class delta_func:
    def __init__(self, k_delta=1, theta_a=1):
        self.k_delta = k_delta
        self.theta_a = theta_a

    def output(self, y1, v) -> float:
        return ((-1) * self.theta_a * np.tanh(self.k_delta * y1 * v))

    def partial_y(self, y1, v) -> float:
        return ((-1) * self.theta_a * self.k_delta * v * (1 - np.power(np.tanh(self.k_delta * y1 * v), 2)))

    def partial_v(self, y1, v) -> float:
        return ((-1) * self.theta_a * self.k_delta * y1 * (1 - np.power(np.tanh(self.k_delta * y1 * v), 2)))

    def dot(self, y1, v, y1_dot, v_dot) -> float:
        return ((self.partial_y(y1, v) * y1_dot) + (self.partial_v(y1, v) * v_dot))


# Frame auxialiary class to be used for path following
def ptf(tangent, v0):
    V = np.zeros(np.shape(tangent))
    V[:, 0] = v0

    for i in range(np.shape(tangent)[1] - 1):
        B = np.cross(np.array([tangent[0, i], tangent[1, i], 0]), np.array([tangent[0, i + 1], tangent[1, i + 1], 0]))
        if np.linalg.norm(B) == 0:
            V[:, i + 1] = V[:, i]
        else:
            B_norm = B / np.linalg.norm(B)
            theta = np.arccos(np.dot(tangent[:, i], tangent[:, i + 1]))
            V[0, i + 1], V[1, i + 1], _ = rotation_around_axis(theta, B_norm).dot(np.array([V[0, i], V[1, i], 0]))
    return V

def rotation_matrix(angle):
    rot_mat = np.array([[np.cos(angle), (-1)*np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    return rot_mat

def rotation_around_axis(angle, axis):
    R11 = np.cos(angle) + (axis[0]**2) * (1 - np.cos(angle))
    R12 = axis[0] * axis[1] * (1 - np.cos(angle)) - np.sin(angle) * axis[2]
    R13 = axis[2] * axis[0] * (1 - np.cos(angle)) + np.sin(angle) * axis[1]
    R21 = axis[0] * axis[1] * (1 - np.cos(angle)) + np.sin(angle) * axis[2]
    R22 = np.cos(angle) + (axis[1]**2) * (1 - np.cos(angle))
    R23 = axis[2] * axis[1] * (1 - np.cos(angle)) - np.sin(angle) * axis[0]
    R31 = axis[0] * axis[2] * (1 - np.cos(angle)) - np.sin(angle) * axis[1]
    R32 = axis[1] * axis[2] * (1 - np.cos(angle)) + np.sin(angle) * axis[0]
    R33 = np.cos(angle) + axis[2]**2 * (1 - np.cos(angle))

    R = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])

    return R

def curvature(X, Y, resolution):

    x_d = np.gradient(X.flatten(), resolution)
    x_dd = np.gradient(x_d, resolution)
    y_d = np.gradient(Y.flatten(), resolution)
    y_dd = np.gradient(y_d, resolution)

    Cc = np.abs(x_dd * y_d - x_d * y_dd) / np.power(x_d**2 + y_d**2, 3/2)

    for i in range(len(Cc)):
        if isnan(Cc[i]):
            Cc[i] = 0

    return Cc


"""
Angle utility section, primarily taken from
https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
"""

# Wrap angle between [-pi, pi]
def angle_wrapper(x):
    x = fmod(x + pi, 2*pi)
    if x < 0:
        x += 2*pi
    return x - pi

# Wrap angle between [0, 2*pi]
def angle_wrapper2pi(x):
    x = fmod(x, 2*pi)
    if x < 0:
        x += 2*pi
    return x

# Difference between two angles
def angle_difference(a, b):
    dif = fmod(a - b + pi, 2*pi)
    if dif < 0:
        dif += 2*pi
    return dif - pi

"""
# This should bisect an angle on the "smaller" side
def bisect_angle(a, b):
    return constrainAngle(a + angle_difference(a, b) * 0.5)

"""

def gamma_difference(gamma0, gamma1):
    if gamma0 >= 0 and gamma0 <= 0.25 and gamma1 >= 0.75 and gamma1 <= 1:
        return (gamma0 + 1 - gamma1)
    elif gamma1 >= 0 and gamma1 <= 0.25 and gamma0 >= 0.75 and gamma0 <= 1:
        return (gamma0 - gamma1 - 1)
    else:
        return (gamma0 - gamma1)

def gamma_change_one(gamma, gamma_list):
    for gamma_item, index in zip(gamma_list, range(len(gamma_list))):
        if gamma_item >= 0.75 and gamma >= 0 and gamma <= 0.25:
            gamma = gamma + 1
        elif gamma >= 0.75 and gamma <= 1 and gamma_item >= 0 and gamma_item <= 0.25:
            gamma_list[index] = gamma_list[index] + 1
    return (gamma, gamma_list)

def gamma_vector_check(gamma_vector):
    gamma_min = gamma_vector[0]
    gamma_max = gamma_vector[0]

    for i in range(len(gamma_vector)):
        if gamma_min >= gamma_vector[i]:
            gamma_min = gamma_vector[i]
        elif gamma_max <= gamma_vector[i]:
            gamma_max = gamma_vector[i]

    if gamma_min >= 0 and gamma_min <= 0.25 and gamma_max >= 0.75 and gamma_max <= 1:
        for i in range(len(gamma_vector)):
            if gamma_vector[i] >= 0 and gamma_vector[i] <= 0.25:
                gamma_vector[i] = gamma_vector[i] + 1

    return gamma_vector