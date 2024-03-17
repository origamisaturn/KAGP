import sys, os
import pickle as pkl
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
sys.path.append(os.path.abspath('core'))

from log_utils import *

# x is v_e and m_dot
# def plot_a_thrust_res(df_eng, v_e_guess, m_dot_guess, m0):
#     thrust_acc = df_eng['acc']
#     t = df_eng['t']

#     x0 = [v_e_guess, m_dot_guess]
#     min_res = lambda x: a_thrust_res(x, t, thrust_acc, m0)

#     x_dev = [1000, 2]
#     x_range_0 = np.linspace(v_e_guess - x_dev[0], v_e_guess + x_dev[0], 5)
#     x_range_1 = np.linspace(m_dot_guess - x_dev[1], m_dot_guess + x_dev[1], 5)
#     nx = len(x_range_0)
#     ny = len(x_range_1)

    # for v_e in x_range_0:
    #     for m_dot in x_range_1:

    # plt.contourf(x, y, zs)

def get_2D_lsq_grid(func, bounds, n_points=5):
    """ Gets gridded points of least squares function.

    Args:
        func: Function used in optimization. Accepts x input.
        bounds: Min and max points of query grid.
        n_points: Number of sample points along each axis

    Returns:
        fig, ax of current figure.
    """
    x_points = np.linspace(bounds[0][0], bounds[0][1], n_points)
    y_points = np.linspace(bounds[1][0], bounds[1][1], n_points)
    n_x = len(x_points)
    n_y = len(y_points)
    xx, yy = np.meshgrid(x_points, y_points, indexing='ij')
    zs = np.zeros((n_x, n_y))
    for xi in range(n_x):
        for yi in range(n_y):
            x_point = [xx[xi, yi], yy[xi, yi]]
            result = func(x_point)
            res = 0.5*sum(result**2)
            zs[xi, yi] = res
    
    return xx, yy, zs

def plot_lsq(func, bounds, n_points=20, levels=20, **kwargs):
    xx, yy, zs = get_2D_lsq_grid(func, bounds, n_points=n_points)
    plt.contourf(xx, yy, zs, **kwargs)

def plot_a_thrust_res(df_log, x0, x_dev, m0, 
                        **kwargs):
    func = lambda x: a_thrust_res(x, df_log['t'], df_log['acc'], m0)
    v_e = x0[0]
    m_dot = x0[1]
    plot_lsq(func, [[v_e - x_dev[0], v_e + x_dev[0]], 
                    [m_dot - x_dev[1], m_dot + x_dev[1]]], 
                    **kwargs)
    plt.colorbar()

def a_thrust_res(x, t, thrust_acc_history, m0):
    v_e = x[0]
    m_dot = x[1]
    tau = m0/m_dot
    calc_a_thrust = v_e/(tau - t)
    res_a_thrust = calc_a_thrust - thrust_acc_history
    return res_a_thrust

def a_thrust_estimate(df_eng, v_e_guess, m_dot_guess, m0):
    thrust_acc = df_eng['acc']
    t = df_eng['t']

    x0 = [v_e_guess, m_dot_guess]
    min_res = lambda x: a_thrust_res(x, t, thrust_acc, m0)
    res = least_squares(min_res, x0, method='lm')
    return res

def mass_res(x, t, mass_history):
    m0 = x[0]
    m_dot = x[1]

    calc_mass = m0 + m_dot*t
    res_mas = calc_mass - mass_history
    return res_mas

def mass_estimate(df_log, m0_guess, m_dot_guess):
    t = df_log['state']['t']
    m = df_log['state']['m']
    x0 = [m0_guess, m_dot_guess]
    
    min_res = lambda x: mass_res(x, t, m)
    res = least_squares(min_res, x0, method='lm')
    return res

def plot_mass_res(df_log, x0, x_dev, n, **kwargs):
    t = df_log['state']['t']
    mass_history = df_log['state']['m']
    func = lambda x: mass_res(x, t, mass_history)
    bounds = []
    for i, x_val in enumerate(x0):
        bounds.append([x_val - x_dev[i], x_val + x_dev[i]])
    plot_lsq(func, bounds, **kwargs)
    plt.color_bar()

def combo_estimate(df_log, df_eng, m0_guess, m_dot_guess, v_e_guess):
    res_mass = mass_estimate(df_log, m0_guess, m_dot_guess)
    m0_est, m_dot_est = tuple(res_mass.x)
    t, thrust_acc_history = df_eng['t'], df_eng['acc']
    a_min_func = lambda x: a_thrust_res([x, m_dot_est], t, thrust_acc_history, m0_est)
    res_a_thrust = least_squares(a_min_func, [v_e_guess], method='lm')
    return res_mass, res_a_thrust

def log_estimates(df_log, df_eng, t_cutoffs, m0_guess, m_dot_guess, v_e_guess):
    res_dict = {'a_thrust_estimate':{'t_cut': [], 'm_dot': [], 'v_e': []},
                'mass_estimate':{'t_cut': [], 'm0': [], 'm_dot': []},
                'combo_estimate':{'t_cut': [], 'm0': [], 'm_dot': [], 'v_e': []}}
    for t in t_cutoffs:
        a_thrust_estimate()
        mass_estimate()
        combo_estimate()
        ...

# def v_e_estimate():
#     ...

if __name__ == '__main__':
    #ScriptKRPC.yaml
    mu = 4.9028e+12 # Moon
    isp = 310
    g0 = 9.80665
    F_thrust = 15.57e+3
    v_e = isp*g0
    m_dot = F_thrust/v_e
    m0 = 3967
    print("v_e: {:.2f}".format(v_e))
    print("m_dot: {:.2f}\n".format(m_dot))


    log_file = "ksp2d_3.pkl"
    with open(log_file, 'rb') as fh:
        log = pkl.load(fh)

    engine_data_file = "engine_est_out_031424"
    with open(engine_data_file, 'rb') as fh:
        eng_dict = pkl.load(fh)

    df_dict = log_to_dataframes(log, mu)
    df_err = dataframe_errors(df_dict)
    df_eng = pd.DataFrame(eng_dict)

    df_test=df_eng[df_eng['t']<500]
    plot_a_thrust_res(df_test, [v_e, m_dot], [1000, 3], m0)
    plt.show()
    # res = a_thrust_estimate(df_test, v_e, m_dot, m0)
    # plot_a_thrust_res(df_test, v_e, m_dot, m0)

        # plt.show()
    # plot_a_thrust_res(df_test, v_e)
    # print(res)