import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from copy import deepcopy

def col_mult(mat1, mat2):
    samples = mat1.shape[1]
    mult = np.zeros(samples)
    for i in range(samples):
        mult[i] = np.dot(mat1[:, i], mat2[:, i])
    return mult

def get_radius(log):
    pos = np.array([log['state']['x'], log['state']['y']])
    radius = np.linalg.norm(pos, axis=0)
    return radius

def get_ground_distance(log, ref_pos):
    # ref_pos determines ground radius and origin angle
    # INCOMPLETE
    radius_ref = np.linalg.norm(ref_pos)
    angle_ref = np.arctan2(ref_pos[1], ref_pos[0])
    distance_ref = radius_ref * angle_ref

def get_r_hat(log):
    pos = np.array([log['state']['x'], log['state']['y']])
    r_hat = pos/np.linalg.norm(pos, axis=0)
    return r_hat

def get_theta_hat(log):
    r_hat = get_r_hat(log)
    rot_mat = np.array([[0, -1], [1, 0]])
    theta_hat = rot_mat@r_hat
    return theta_hat

def get_r_dot(log):
    v = np.array([log['state']['vx'], log['state']['vy']])
    r_hat = get_r_hat(log)
    r_dot = col_mult(r_hat, v)
    return r_dot

def get_v_theta(log):
    v = np.array([log['state']['vx'], log['state']['vy']])
    pos = np.array([log['state']['x'], log['state']['y']])
    theta_hat = get_theta_hat(log)
    v_theta = col_mult(theta_hat, v)
    return v_theta

def get_r_dot_dot(log):
    #acc = get_acc(log)
    #r_hat = get_r_hat(log)
    #r_dot_dot = col_mult(acc, r_hat)
    r_dot = get_r_dot(log)
    t = log['state']['t']
    r_dot_dot = np.gradient(r_dot, t)
    return r_dot_dot

def get_a_theta(log):
    #acc = get_acc(log)
    #theta_hat = get_theta_hat(log)
    #a_theta = col_mult(acc, theta_hat)
    v_theta = get_v_theta(log)
    t = log['state']['t']
    a_theta = np.gradient(v_theta, t)
    return a_theta

def get_acc(log):
    vx = log['state']['vx']
    vy = log['state']['vy']
    t = log['state']['t']
    v = np.array((vx, vy))
    acc = np.gradient(v, t, axis=1)
    return acc

def get_orbital_elements(log):
    ...

def get_alpha(log):
    # the input and output already have alpha
    ...

def plot_state(log):
    # based on the amount of variables in the log, choose
    # the shape of the subplots. Lets do variable rows, 
    # fixed columns. title based on variable name. the only label.
    # OK this is good.
    t = log['state']['t']
    y_var_names = list(log['state'].keys())
    y_var_names.remove('t')
    vars = log['state']
    # -1 to ignore the 't' key
    fig, axs = plot_vars(vars, t, 3, keys=y_var_names)
    return fig, axs

def plot_problem_outputs(log):
    outputs = log['outputs']
    t = log['inputs']['pitch_query.t']
    fig, ax = plot_vars(outputs, t, 3)
    return fig, ax

def plot_problem_inputs(log):
    inputs = log['inputs']
    t = log['inputs']['pitch_query.t']
    fig, ax = plot_vars(inputs, t, 3)
    return fig, ax

def plot_vars(vars, t, columns, keys=None):
    if keys:
        var_names = keys
    else:
        var_names = list(vars.keys())
    columns = 3
    plot_total = len(var_names)
    rows = int(np.ceil(plot_total/columns))
    fig, axs = plt.subplots(rows, columns)
    for i in range(rows):
        for j in range(columns):
            plot_index = i*columns + j
            if plot_index >= plot_total:
                continue
            else:
                var_name = var_names[plot_index]
                var_val = vars[var_name]
                print("var_name: {}".format(var_name))
                # ignore _debug dictionary
                if type(var_val) == type(dict()):
                    continue

                axs[i, j].plot(t, var_val)
                axs[i, j].set_title(var_name)
    return fig, axs

def plot_derived_state(log):
    # also n by 4, based on my custom functions.
    t = log['state']['t']

    ref_pos = [log['state']['x'][0], log['state']['y'][0]]
    r = get_radius(log)
    #s = get_ground_distance(log, ref_pos)
    r_hat = get_r_hat(log)
    theta_hat = get_theta_hat(log)
    r_dot = get_r_dot(log)
    v_theta = get_v_theta(log)
    r_dot_dot = get_r_dot_dot(log)
    a_theta = get_a_theta(log)
    acc = get_acc(log)
    acc_x = acc[0]
    acc_y = acc[1]
    vars = {"radius": r, "ground_distance": t, "r_dot": r_dot, "v_theta":v_theta,
            "r_dot_dot":r_dot_dot, "a_theta":a_theta, "acc_x":acc_x,
            "acc_y":acc_y}
    fig, axs = plot_vars(vars, t, 3)
    return fig, axs

def outputs_dataframe(log):
    df_log = pd.DataFrame(log['inputs'])
    
    return df_log

def log_to_dataframes(log):
    dataframes = {}
    formatted_log = format_log(log)
    interpolate_times = formatted_log['inputs']['pitch_query.t']
    interp_state = interpolate_state(formatted_log, interpolate_times)

    dataframes['outputs'] = pd.DataFrame(formatted_log['outputs'])
    dataframes['inputs'] = pd.DataFrame(formatted_log['inputs'])
    dataframes['state'] = pd.DataFrame(interp_state)

    return dataframes


def format_log(log):
    """ Convert to format for dataframe 
    
    Makes everything into a 1-D list. """
    new_log = {}

    inputs = log['inputs']
    outputs = log['outputs']
    state = log['state']

    new_log['inputs'] = format_prob_log(inputs)
    new_log['outputs'] = format_prob_log(outputs)
    new_log['state'] = deepcopy(state)

    return new_log

def format_prob_log(prob_log):
    new_prob_log = {}
    for var_name, var_val in prob_log:
        var_len = np.shape(var_val)[1]
        if type(var_val) == type(dict()):
            for dict_key, dict_val in var_val:
                new_key = var_name + '.' + dict_key
                new_val = dict_val
                new_prob_log[new_key] = new_val
        elif var_len != 1:
            for i in range(var_len):
                new_key = "{}[{}]".format(var_name, i)
                new_val = list(var_val[:, i])
                new_prob_log[new_key] = new_val
        else:
            new_val = list(var_val[:, 0])
            new_prob_log[var_name] = new_val
    return new_prob_log
            
def interpolate_state(log, interpolation_times):
    new_state = {}
    state = log['state']
    # Hope that state t is monotonically increasing
    t_orig = state['t']

    for var_name, var_val in state:
        if var_name == 't':
            new_val = deepcopy(interpolation_times)
        else:
            new_val = np.interp(interpolation_times, t_orig, var_val)
        new_state[var_name] = new_val

    return new_state


# def flatten_outputs(log):
#     # dicts are converted to a single name
#     # multiple dimensions are split
#     # 2d is converted to 1d array
#     outputs = log['outputs']
#     for var_name, var_data in outputs:
#         np_var_data = np.array(var_data)
#         if np.shape[1] 
#     return flat_outputs