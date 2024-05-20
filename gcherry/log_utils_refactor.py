import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from gcherry.transform import rcn2global_rot, global2topo_rot, get_ra_decl
from copy import deepcopy

from poliastro.core.elements import rv2coe

""" All args and returns are assumed to be in the global frame. The global frame
is an inertial frame whose origin is fixed at the center of the major
body.
"""

def col_mult(mat1, mat2):
    """ multiply 2d arrays column by column. """
    samples = mat1.shape[1]
    mult = np.zeros(samples)
    for i in range(samples):
        mult[i] = np.dot(mat1[:, i], mat2[:, i])
    return mult

def get_radius(pos):
    """ Get radius.
    
    Args:
        pos: 3xN array of position in the global frame.
        
    Returns:
        Length N 1-D array of radius at each column of pos.
        
    """
    radius = np.linalg.norm(pos, axis=0)
    return radius

def get_ground_distance(log, ref_pos):
    # ref_pos determines ground radius and origin angle
    # INCOMPLETE
    radius_ref = np.linalg.norm(ref_pos)
    angle_ref = np.arctan2(ref_pos[1], ref_pos[0])
    distance_ref = radius_ref * angle_ref

def get_r_hat(pos):
    """ Get radial unit vector.

    Args:
        pos: 3xN array of position in the global frame.

    Returns:
        3xN array representing radial unit vector at each column of 
        pos.

    """
    r_hat = pos/np.linalg.norm(pos, axis=0)
    return r_hat

def get_theta_hat(pos, vel):
    """ Get circumferential unit vector. 
    
    Args:
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
        
    Returns:
        3xN array representing circumferential unit vector at each
        column of pos and vel.

    """
    N = pos.shape[1]
    theta_hat = np.zeros((3, N))
    for i in range(N):
        theta_hat[:, i] = rcn2global_rot(pos, vel)@np.array([0, 1, 0])
    return theta_hat

def get_r_dot(pos, vel):
    """ Get rate of change of radius wrt time. 
    
    Args:
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
        
    Returns:
        Length N 1-D array representing rate of change of radius wrt time at each
        column of pos and vel.

    """
    r_hat = get_r_hat(pos)
    r_dot = col_mult(r_hat, vel)
    return r_dot

def get_v_theta(pos, vel):
    """ Get velocity along circumferential unit vector. 
    
    Args:
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
    Returns:
        Length N 1-D array representing velocity along circumferential unit vector
        at each column of pos and vel. 
    
    """
    theta_hat = get_theta_hat(pos, vel)
    v_theta = col_mult(theta_hat, vel)
    return v_theta

def get_r_dot_dot(t, pos, vel):
    """ Get rate of change of r_dot wrt time.
    
    Args:
        t: N-length 1-D array of time.
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
    
    Returns:
        Length N 1-D array representing rate of change of r_dot wrt time.
        
    """
    r_dot = get_r_dot(pos, vel)
    r_dot_dot = np.gradient(r_dot, t)
    return r_dot_dot

def get_a_theta(t, pos, vel):
    """ Get rate of change of circumferential velocity wrt time.
    
    Args:
        t: N-length 1-D array of time.
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.

    Returns:
        Length N 1-D array representing rate of change of v_theta wrt time.

    """
    v_theta = get_v_theta(pos, vel)
    a_theta = np.gradient(v_theta, t)
    return a_theta

def get_acc(t, vel):
    """ Get acceleration.
    
    Args:
        t: N-length 1-D array of time.
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
    
    Returns:
        3xN array representing acceleration at each t element.

    """
    acc = np.gradient(vel, t, axis=1)
    return acc

def get_gravity(pos, mu):
    """ Get acceleration due to gravity. Assumes spherical major body.
    
    Args:
        pos: [m] 3xN array of position in the global frame.
        mu: [m**3 s**-2] Gravitational parameter 
    
    Returns:
        3xN array representing acceleration due to gravity at each pos
        column.
    
    """
    r_hat = get_r_hat(pos)
    r = get_radius(pos)
    a_g = -mu/r**2 * r_hat
    return a_g

def get_non_gravity_acc(t, pos, vel, mu):
    """ Get acceleration not due to gravity.
    
    Args:
        t: [s] N-length 1-D array of time.
        pos: [m] 3xN array of position in the global frame.
        vel: [m/s] 3xN array of velocity in the global frame.
        mu: [m**3 s**-2] Gravitational parameter 
    
    Returns:
        3xN array representing acceleration due to thrust.
    
    """
    a_g = get_gravity(pos, mu)
    acc = get_acc(t, vel)
    perturb = acc - a_g
    return perturb

def get_non_gravity_acc_mag(t, pos, vel, mu):
    """ Get magnitude of acceleration due to sources other than gravity.
    
    Args:
        t: [s] N-length 1-D array of time.
        pos: [m] 3xN array of position in the global frame.
        vel: [m/s] 3xN array of velocity in the global frame.
        mu: [m**3 s**-2] Gravitational parameter 

    Returns:
        Length N 1-D array representing rate of change of v_theta wrt time.

    """
    perturb = get_non_gravity_acc(t, pos, vel, mu)
    perturb_mag = np.linalg.norm(perturb, axis=0)
    return perturb_mag

def get_orbital_elements(pos, vel, mu):
    """ Get orbital elements.

    Args:
        pos: [m] 3xN array of position in the global frame.
        vel: [m/s] 3xN array of velocity in the global frame.
        mu: [m**3 s**-2] Gravitational parameter 

    Returns:
        6xN array of orbital elements at each column of pos. Each entry
        contains:
            a: Semi-major axis
            e: Eccentricity
            i: [rad.] Inclination
            lan: [rad.] Longitude of ascending node
            argp: [rad.] Argument of periapsis
            nu: [rad.] True anomaly
    
    """
    N = np.shape(pos)[1]
    orbital_elements = np.zeros((6, N))
    m_to_km = 1/1e3
    k = mu * m_to_km**3
    for i in range(N):
        r = pos[:, i] * m_to_km
        v = vel[:, i] * m_to_km
        # p [km], ecc, inc [rad.], raan [rad.], argp [rad.], nu [rad.]
        p, ecc, inc, raan, argp, nu = rv2coe(k, r, v)
        # convert semi-latus rectum [km] to semi-major axis [m]
        a = p/(1 - ecc**2) * 1/m_to_km
        orbital_elements[:, i] = [a, ecc, inc, raan, argp, nu]
    return orbital_elements


def get_thrust_pitch(t, pos, vel, mu):
    """ Get pitch of thrust, assuming get_non_gravity_acc() results in 
    thrust acceleration.
    
    Args:
        t:
        pos:
        vel:
        mu:

    Returns:
        Length N 1-D array representing pitch of thrust relative to
        local horizon.

    """
    thrust_acc = get_non_gravity_acc(t, pos, vel, mu)
    ra, decl = get_ra_decl(pos)
    thrust_acc_topo = global2topo_rot(ra, decl)@thrust_acc
    alpha = np.arctan2(-thrust_acc_topo[2, :], thrust_acc[:2, :])
    return alpha

def get_target_normal_position(pos, target_lan, target_inc):
    ...

def get_target_normal_velocity():
    ...

def get_target_normal_acceleration():
    ...

    
def get_time_steps(t):
    """ Get step between each time entry. 
    
    Args:
        t: 
    
    Returns:
        Length N 1-D array of time steps.

    """
    time_steps = np.zeros(len(t))
    for i, t_val in enumerate(t):
        if i == 0:
            time_steps[i] = 0
        else:
            time_steps[i] = t_val - t[i-1]
    return time_steps

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
    fig.suptitle('State')
    return fig, axs

def plot_problem_outputs(log):
    outputs = log['outputs']
    t = log['inputs']['pitch_query.t']
    fig, ax = plot_vars(outputs, t, 3)
    fig.suptitle('Problem Outputs')
    return fig, ax

def plot_problem_inputs(log):
    inputs = log['inputs']
    t = log['inputs']['pitch_query.t']
    fig, ax = plot_vars(inputs, t, 3)
    fig.suptitle('Problem Inputs')
    return fig, ax

def plot_vars(vars, t, columns=3, keys=None):
    if keys:
        var_names = keys
    else:
        var_names = list(vars.keys())
    plot_total = len(var_names)
    rows = int(np.ceil(plot_total/columns))
    fig, axs = plt.subplots(rows, columns, sharex=True)
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

def plot_derived_state(log, mu):
    # also n by 4, based on my custom functions.
    t = log['state']['t']

    derived_state = get_derived_state(log, mu)
    fig, axs = plot_vars(derived_state, t, 3)
    fig.suptitle('Derived State')
    return fig, axs

def get_derived_state(log, mu):
    t = log['state']['t']
    r = get_radius(log)
    #s = get_ground_distance(log, ref_pos)
    r_dot = get_r_dot(log)
    v_theta = get_v_theta(log)
    r_dot_dot = get_r_dot_dot(log)
    a_theta = get_a_theta(log)
    acc = get_acc(log)
    thrust_acc_mag = get_thrust_acc_mag(log, mu)
    alpha = get_alpha(log, mu)
    acc_x = acc[0]
    acc_y = acc[1]
    vars = {"alpha": alpha, "thrust_acc": thrust_acc_mag, "radius": r, "r_dot": r_dot, "v_theta":v_theta,
            "r_dot_dot":r_dot_dot, "a_theta":a_theta, "acc_x":acc_x,
            "acc_y":acc_y, "t": t}
    
    return vars



### LOG TO DATAFRAME ###

def log_to_dataframes(log, mu):
    dataframes = {}

    formatted_log = format_log(log)
    interpolate_times = formatted_log['inputs']['pitch_query.t']
    interp_state = interpolate_state(formatted_log, interpolate_times)
    formatted_log['state'] = interp_state
    derived_state = get_derived_state(formatted_log, mu)

    dataframes['outputs'] = pd.DataFrame(formatted_log['outputs'])
    dataframes['inputs'] = pd.DataFrame(formatted_log['inputs'])
    dataframes['state'] = pd.DataFrame(formatted_log['state'])
    dataframes['derived'] = pd.DataFrame(derived_state)

    return dataframes

# Consider moving interpolation here, also adding error calc
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

def dataframe_errors(df_dict):
    inp = df_dict['inputs']
    out = df_dict['outputs']
    der = df_dict['derived']
    t = df_dict['derived']['t']
    t_step = get_time_steps(df_dict['derived']['t'])
    r_dot_dot_error = (df_dict['derived']['r_dot_dot'] - 
                       df_dict['outputs']['pitch_query._debug.r_dot_dot'])
    r_dot_error = (df_dict['derived']['r_dot'] - 
                       df_dict['outputs']['pitch_query._debug.r_dot'])
    r_error = (df_dict['derived']['radius'] - 
                       df_dict['outputs']['pitch_query._debug.r'])
    thrust_acc_error = (df_dict['derived']['thrust_acc'] -
                        df_dict['outputs']['pitch_query._debug.a_thrust'])
    thrust_alpha_error = (df_dict['derived']['alpha'] - 
                          df_dict['outputs']['pitch_query.alpha'])
    
    return pd.DataFrame(
        {'t': t,
         't_step': t_step,
         'r_dot_dot_err': r_dot_dot_error,
         'r_dot_err': r_dot_error,
         'r_err': r_error,
         'thrust_acc_err': thrust_acc_error,
         'thrust_alpha_error': thrust_alpha_error}
    )

def plot_dataframe_errors(df_dict):
    df_err = dataframe_errors(df_dict)
    plot_vars(df_err, df_err['t'], 2, keys=list(df_err.columns))


def format_prob_log(prob_log):
    new_prob_log = {}
    for var_name, var_val in prob_log.items():
        if type(var_val) == type(dict()):
            for dict_key, dict_val in var_val.items():
                new_key = var_name + '.' + dict_key
                new_val = dict_val
                new_prob_log[new_key] = new_val
        else:
            var_len = np.shape(var_val)[1]
            np_var_val = np.array(var_val)
            if var_len != 1:
                for i in range(var_len):
                    new_key = "{}[{}]".format(var_name, i)
                    new_val = list(np_var_val[:, i])
                    new_prob_log[new_key] = new_val
            else:
                new_val = list(np_var_val[:, 0])
                new_prob_log[var_name] = new_val
    return new_prob_log
            
def interpolate_state(log, interpolation_times):
    new_state = {}
    state = log['state']
    # Hope that state t is monotonically increasing
    t_orig = state['t']

    for var_name, var_val in state.items():
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