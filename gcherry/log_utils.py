import numpy as np
import matplotlib.pyplot as plt

from gcherry.transform import (
    rcn2global_rot,
    global2topo_rot, 
    get_ra_decl, 
    perifocal2global_rot, 
    body2global_rot, 
    global2pcf_rot, 
    global2perifocal_rot)

# TODO: Would be nice to get rid of this, poliastro is holding the python
# version back to 3.10.x.
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
        column of pos and vel. Will be NaN if pos and vel are parallel.

    """
    N = pos.shape[1]
    theta_hat = np.zeros((3, N))
    for i in range(N):
        theta_hat[:, i] = (rcn2global_rot(pos[:, i], vel[:, i])
                           @np.array([0, 1, 0]))
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
        at each column of pos and vel. Positive by definition.
    
    """
    # theta_hat = get_theta_hat(pos, vel)
    # v_theta = col_mult(theta_hat, vel)
    # more robust to theta_hat not existing
    r_dot = get_r_dot(pos, vel)
    v_mag = np.linalg.norm(vel, axis=0)
    v_theta = np.sqrt(v_mag**2 - r_dot**2)
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
            a: [m] Semi-major axis
            e: Eccentricity
            i: [rad.] Inclination
            lan: [rad.] Longitude of ascending node
            argp: [rad.] Argument of periapsis
            nu: [rad.] True anomaly

        If velocity vector is 0 or colinear w/ radius, then orbital
        elements are NaN for that column.
    
    """
    N = np.shape(pos)[1]
    orbital_elements = np.zeros((6, N))
    m_to_km = 1/1e3
    k = mu * m_to_km**3
    for i in range(N):
        r = pos[:, i] * m_to_km
        v = vel[:, i] * m_to_km

        # If orbital elements exist
        if (np.linalg.norm(v) > 1e-8 and
            np.linalg.norm(np.cross(r, v)) > 1e-8):
        # p [km], ecc, inc [rad.], raan [rad.], argp [rad.], nu [rad.]
            p, ecc, inc, raan, argp, nu = rv2coe(k, r, v)
            # convert semi-latus rectum [km] to semi-major axis [m]
            a = p/(1 - ecc**2) * 1/m_to_km
            orbital_elements[:, i] = [a, ecc, inc, raan, argp, nu]
        else:
            orbital_elements[:, i] = [np.nan for i in range(6)]
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
    thrust_acc_topo = np.zeros(np.shape(thrust_acc))
    for i in range(thrust_acc.shape[1]):
        ra, decl = get_ra_decl(pos[:, i])
        thrust_acc_topo[:, i] = global2topo_rot(ra, decl)@thrust_acc[:, i]
    alpha = np.arctan2(-thrust_acc_topo[2, :], np.linalg.norm(thrust_acc_topo[:2, :], axis=0))
    return alpha

def get_projected_true_anomaly(pos, target_lan, target_inc, target_argp):
    """ Get true anomaly of position as projected onto target orbit.
    
    Args:
        pos: [m] 3xN array of global position.
        target_lan: [rad.] N-length 1-D array of target longitude of ascending 
            node.
        target_inc: [rad.] N-length 1-D array of target inclination.
        target_argp: [rad.] N-length 1-D array of target argument of periapsis.

    Returns:
        Length N 1-D array of true anomaly [rad.] as projected onto 
        target orbit. Range [-pi, pi]

    """
    N = pos.shape[1]
    nu_proj = np.zeros(N)
    for i in range(N):
        pos_perifocal = global2perifocal_rot(target_lan[i], target_inc[i], target_argp[i])@pos[:, i]
        nu_proj[i] = np.arctan2(pos_perifocal[1], pos_perifocal[0])
    return nu_proj

def get_thrust_acc_PCF(pos, thrust_pitch, thrust_yaw, m, target_lan, target_inc, F_thrust_max):
    """ Get thrust acceleration in Plane Control Frame.
     
    Calculates using method in rocket_ode().

    Args:
        pos: [m] 3xN array of global position.
        thrust_pitch: [rad.] N-length 1-D array of thrust pitch.
        thrust_yaw: [rad.] N-length 1-D array of thrust yaw.
        m: [kg] N-length 1-D array of mass.
        target_lan: [rad.] N-length 1-D array of target longitude of
          ascending node.
        target_inc: [rad.] N-length 1-D array of target inclination.
        F_thrust_max: [N] Maximum thrust

    Returns:
        3xN array of acceleration [m/s**2] due to thrust in Plane 
        Control Frame.

    """
    F_thrust = F_thrust_max
    thrust_vector_body = np.array([F_thrust, 0, 0])
    N = len(m)
    thrust_acc_pcf = np.zeros((3, N))
    for i in range(N):
        thrust_vector_global = (body2global_rot(0, thrust_pitch[i], thrust_yaw[i], pos[:, i]) 
                                @ thrust_vector_body)
        thrust_vector_pcf = global2pcf_rot(pos[:, i], target_lan[i], target_inc[i])@thrust_vector_global
        thrust_acc_pcf[:, i] = thrust_vector_pcf/m[i]
    return thrust_acc_pcf

def get_theta_hat_PCF(pos, vel, target_lan, target_inc):
    """ Get circumferential unit vector in Plane Control Frame. 
    
    Args:
        pos: 3xN array of position in the global frame.
        vel: 3xN array of velocity in the global frame.
        target_lan: [rad.] N-length 1-D array of target longitude of
          ascending node.
        target_inc: [rad.] N-length 1-D array of target inclination.
        
    Returns:
        3xN array representing circumferential unit vector at each
        column of pos and vel.

    """
    N = pos.shape[1]
    theta_hat = np.zeros((3, N))
    for i in range(N):
        theta_hat[:, i] = (
            global2pcf_rot(pos[:,i], target_lan[i], target_inc[i])
            @rcn2global_rot(pos[:, i], vel[:, i])
            @np.array([0, 1, 0]))
    return theta_hat

def get_target_normal_position(pos, target_lan, target_inc):
    target_normal_vec = (perifocal2global_rot(target_lan, target_inc, 0) @ 
                        np.array([0, 0, 1]))
    target_normal_position = target_normal_vec@pos
    return target_normal_position

def get_target_normal_velocity(vel, target_lan, target_inc):
    target_normal_vec = (perifocal2global_rot(target_lan, target_inc, 0) @ 
                        np.array([0, 0, 1]))
    target_normal_velocity = target_normal_vec@vel
    return target_normal_velocity

def get_target_normal_acceleration(t, vel, target_lan, target_inc):
    target_normal_velocity = get_target_normal_velocity(vel, target_lan, target_inc)
    target_normal_acceleration = np.gradient(target_normal_velocity, t)
    return target_normal_acceleration

    
def get_time_steps(t):
    """ Get step between each time entry. 
    
    Args:
        t: 
    
    Returns:
        Length N 1-D array of time steps. 0th element is 0, 1st element
        is t[1] - t[0].

    """
    time_steps = np.zeros(len(t))
    for i, t_val in enumerate(t):
        if i == 0:
            time_steps[i] = 0
        else:
            time_steps[i] = t_val - t[i-1]
    return time_steps

def plot_vars(vars, t, columns=3, keys=None, plotkwargs=None):
    """ Plot several variables on a grid.
    
    Args:
        vars: Dictionary whose entries contain 1-dimensional 
            array-likes to plot on y-axes. All entries must be same length.
        t: 1-dimensional array-like containing x-axis variables.
        columns: Number of columns in plot grid.
        keys: Keys in vars to plot.

    Returns:
        2-tuple containing matplotlib figure and axes.
        
    """
    if plotkwargs is None:
        plotkwargs={}
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

                axs[i, j].plot(t, var_val, **plotkwargs)
                axs[i, j].set_title(var_name)
    return fig, axs

def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol
    
def interp_table(x, xkey, table):
    """ Interpolate dictionary or dataframe.
    
    Args:
        x: Value to interpolate at.
        xkey: Key in table containing list or series to interpolate over.
        table: dict of lists (of equal dimension) or a dataframe object.
    
    Returns:
        dictionary containing each key in table and the interpolated
        value at x.
    
    """
    new_table = {}
    for key in table:
        new_table[key] = np.interp(x, table[xkey], table[key])
    return new_table