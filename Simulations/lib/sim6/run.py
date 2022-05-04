"""

Author: Francisco Branco
Created: 17/11/2021
Description: Formation Control Simple example

"""



import numpy as np
from math import pi

import pathgeneration as pg
import lib.sim6.systembuild as sb
import lib.sim6.plot as plotting


def simulation():
    # Path parameters
    resolution = 40

    start_target = 0
    position_target = np.array([0, 0])
    orientation_target = -pi/2
    size_target = 15.0
    arc_target = 2*pi
    radius_target = size_target

    start = 0
    position0 = np.array([0, 0])
    position1 = np.array([0, 0])
    orientation = -pi/2
    size = 7.5
    arc = 2*pi
    radius = size

    # Path creation
    p_target0 = pg.Path()
    circle_target0 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target - 10, start_target)
    p_target0.append_path(circle_target0)

    p_target1 = pg.Path()
    circle_target1 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target - 5, start_target)
    p_target1.append_path(circle_target1)

    p_target2 = pg.Path()
    circle_target2 = pg.Circle(resolution, position_target, orientation_target, arc_target, radius_target, start_target)
    p_target2.append_path(circle_target2)

    p_follower0 = pg.Path()
    circle_follower0 = pg.Circle(resolution, position0, orientation, arc, radius, start)
    p_follower0.append_path(circle_follower0)

    p_follower1 = pg.Path()
    circle_follower1 = pg.Circle(resolution, position1, 0, arc, radius, start)
    p_follower1.append_path(circle_follower1)


    # Time parameters
    total_time = 350
    num_points = total_time * 20
    T, dt = np.linspace(start=0, stop=total_time, num=num_points, retstep=True)
    
    # Vehicle initial conditions
    x = -10
    y = -10
    theta_m = 0
    s = 0

    pf_params = {
        "gamma": 1,
        "k1": 1,
        "k2": 0.3,
        "k_delta": 1,
        "theta_a": 0.8
    }

    cpf_params_target = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": 0.25,
        "k_csi1": 0.25,
        "k_csi2": 0.25,
        "norm0": p_target0.total_distance,
        "norm1": p_target1.total_distance,
        "norm2": p_target2.total_distance,
        "speed_profile0": p_target0.total_distance / (p_target2.total_distance / 0.25),
        "speed_profile1": p_target1.total_distance / (p_target2.total_distance / 0.25),
        "speed_profile2": 0.25
    }

    cpf_params_follower = {
        "c0": 0.01, #0.01
        "c1": 0.5, #0.5
        "c2": 1, #1
        "l": 1,
        "k": 1.1584,
        "epsilon": 0.1,
        "epsilon0": np.power(10.0, -4),
        "theta": 0.99,
        "k_csi0": 0.5,
        "k_csi1": 0.5,
        "norm0": p_follower0.total_distance,
        "norm1": p_follower1.total_distance,
        "speed_profile0": 1,
        "speed_profile1": 1
    }


    # System creation along with initial conditions
    auv_system = sb.DoubleASVMPFETCOnTripleAUV(
        p_target0,
        p_target1,
        p_target2,
        p_follower0,
        p_follower1,
        pf_params=pf_params,
        cpf_params_target=cpf_params_target,
        cpf_params_follower=cpf_params_follower,
        etc_type="Time",
        history=True,
        dt=dt
    )

    ic = {
        "x_target0": -4,
        "y_target0": -5,
        "theta_m_target0": theta_m,
        "s_target0": s,
        "x_target1": -4,
        "y_target1": -10,
        "theta_m_target1": theta_m,
        "s_target1": s,
        "x_target2": -4,
        "y_target2": -15,
        "theta_m_target2": theta_m,
        "s_target2": s,
        "x_follower0": 0,
        "y_follower0": -12,
        "theta_m_follower0": theta_m,
        "s_follower0": s,
        "x_follower1": 5,
        "y_follower1": -13,
        "theta_m_follower1": theta_m,
        "s_follower1": s
    }
    auv_system.set_initial_conditions(ic)

    # Run the system
    for t in T:
        auv_system.update(t)


    # Plotting
    paths = {"p_target0": p_target0, "p_target1": p_target1, "p_target2": p_target2, "p_follower0": p_follower0, "p_follower1": p_follower1}
    # Get past values for plotting
    past_values = auv_system.past_values()

    plotting.plot(paths, num_points, total_time, resolution, T, past_values)