import numpy as np
from scipy.linalg import block_diag
from scipy.io import loadmat


def load_constants_Multicopter(psi, N_sim, Te, Thrust_const):
    # Multicopter system constants (see paper)
    # This is the continuous model
    A = np.concatenate((np.concatenate((np.zeros((3, 3)), np.identity(3)), axis=1),
                        np.concatenate((np.zeros((3, 3)), np.zeros((3, 3))), axis=1)),
                       axis=0)
    B = np.concatenate((np.zeros((3, 3)), np.identity(3)), axis=0)
    # discrete model
    Ad = np.concatenate(
        (
            np.concatenate((np.identity(3), np.identity(3) * Te), axis=1),
            np.concatenate((np.zeros((3, 3)), np.identity(3)), axis=1)
        ), axis=0)
    Bd = np.concatenate((np.identity(3) * 0.5 * Te ** 2, np.identity(3) * Te), axis=0)
    (height, width) = np.shape(B)

    g = 9.81
    m = 0.03  # Crazyflie 2.0 (~0.027) + Lighthouse deck (~0.03)
    v_equilibrium = [g, 0, 0]
    u_equilibrium = [g, 0, 0]

    T_limit = 2 * g
    epsilon = np.pi / 18  # 10 degrees

    lb_T = 0
    ub_T = T_limit
    lb_phi = -epsilon
    ub_phi = epsilon
    lb_theta = -epsilon
    ub_theta = epsilon

    lower_bounds = [lb_T, lb_phi, lb_theta]
    upper_bounds = [ub_T, ub_phi, ub_theta]

    delta_T = T_limit
    delta_phi = epsilon
    delta_theta = epsilon
    lower_delta_bounds = [-delta_T, -delta_phi, -delta_theta]
    upper_delta_bounds = [delta_T, delta_phi, delta_theta]

    yaw = psi

    plant = {'Te': Te, 'A': A, 'B': B, 'dx': height, 'du': width,
             'T_limit': T_limit, 'lower_bounds': lower_bounds,
             'upper_bounds': upper_bounds, 'lower_delta_bounds': lower_delta_bounds,
             'upper_delta_bounds': upper_delta_bounds,
             'g': g, 'm': m, 'Thrust constant': Thrust_const,
             'v_equilibrium': v_equilibrium, 'yaw': yaw, 'u_equilibrium': u_equilibrium,
             'Ad': Ad, 'Bd': Bd,
             'Tmax': T_limit, 'epsilon_max': epsilon,
             'Nsim': N_sim}

    return plant
