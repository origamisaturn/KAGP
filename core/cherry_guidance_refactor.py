import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

def rot_mat_2d(theta):
    cos = math.cos
    sin = math.sin
    rot_mat = np.array([[cos(theta), -sin(theta)],
        [sin(theta), cos(theta)]])
    return rot_mat

def angle_between_vectors(v1, v2):
    cos_theta = np.dot(v1, v2)/(np.linalg.norm(v1) * np.linalg.norm(v2))
    return np.acos(cos_theta)

def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol


def get_2D_local_axes(x, v):
    """ Calculates r_hat and theta_hat.
    
    Args:
        x: [m] (len(x) == 2) 2D position in inertial frame. Origin is 
            at gravitational body center.
        v: [m/s] (len(v) == 2) 2D velocity in inertial frame.
    
    Returns:
        (r_hat, theta_hat): Unit vectors representing the axes of the
          local coordinate system, where r_hat is radial and theta_hat
          is the local horizon in the direction of travel.

    Raises:
        BaseException
    """
    r = np.linalg.norm(x)
    r_hat = x/r

    candidate_theta_hat_1 = rot_mat_2d(math.pi/2)@r_hat
    candidate_theta_hat_2 = rot_mat_2d(-math.pi/2)@r_hat

    # When v is aligned with x and theta_hat is impossible to
    # calculate, choose this default direction.
    if (almost_equal(v, [0,0]) or
        almost_equal(angle_between_vectors(x, v), 0) or 
        almost_equal(angle_between_vectors(x, v), math.pi)):
        return r_hat, candidate_theta_hat_1
    else:
        v_hat = v/np.linalg.norm(v)
        if np.dot(v_hat, candidate_theta_hat_1) > 0:
            return r_hat, candidate_theta_hat_1
        if np.dot(v_hat, candidate_theta_hat_2) > 0:
            return r_hat, candidate_theta_hat_2
        else:
            raise BaseException("Something unexpected happened.")

def calc_2D_local_velocity(x, v):
    r_hat, theta_hat = get_2D_local_axes(x, v)
    r_dot = np.dot(v, r_hat)
    v_theta = np.dot(v, theta_hat)
    if v_theta < 0:
        raise ValueError("Calculated v_theta is negative.")
    return r_dot, v_theta

class RadialControl(om.ExplicitComponent):
    """
    Component containing radial rate control block.

    Inputs:
        x: [m] (len(x) == 2) 2D position in inertial frame. Origin is 
            at gravitational body center.
        v: [m/s] (len(v) == 2) 2D velocity in inertial frame.
        sample_t: [s] Time at which x and v were collected.
        r_dot_T: [m/s] Target final radial velocity.
        r_T: [m] Target final radial position.
        mu: [m^3/s^2] Gravitational parameter.
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft.

    Outputs:
        a0, a1, a2, c1, c2: Coefficients used to describe radial
            acceleration over time.
        g_eff: [m/s^2] Effective gravity at time sample_t.

    """

    def setup(self):
        # 2D state.
        self.add_input('x', val=np.zeros((2)))
        self.add_input('v', val=np.zeros((2)))
        input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'T', 'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)

        output_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'g_eff']
        for name in output_names:
            self.add_output(name, val=0.0)

    def setup_partials(self):
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        # This assumes thrust on since t = 0
        
        x0 = np.array(inputs['x'])
        v0 = np.array(inputs['v'])
        t = inputs['sample_t'][0]
        T = inputs['T'][0]
        mu = inputs['mu'][0]
        v_e = inputs['v_e'][0]
        m_dot = inputs['m_dot'][0]
        m0 = inputs['m0'][0]
        tau = m0/m_dot
        r_dot_T = inputs['r_dot_T'][0]
        r_T = inputs['r_T'][0]
        T_go = T-t

        r_dot_0, v_theta_0 = calc_2D_local_velocity(x0, v0)

        # Calculate F matrix
        # Compute values of ai
        # a0 is equivalent to thrust at terminal time
        # This might assume t=0.
        a0 = v_e/(tau - T)
        a1 = -a0**2/v_e
        a2 = a0**3/v_e**2

        f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
        f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
        f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
        f12 = f21

        # Ax = b, where A is F vector, b is boundary conditions, x is 
        # [c1, c2]
        r0 = np.linalg.norm(x0)

        A = np.array([[f11, f12], [f21, f22]])
        b = np.array([[r_dot_T - r_dot_0], 
                    [r_T - (r0 + r_dot_0*T_go)]])
        c = np.linalg.solve(A, b).reshape(2)

        c1 = c[0]
        c2 = c[1]

        # Compute effective g_eff at current time. This is a function of t,
        # but we will assume it is static for each iteration.
        g_eff = -mu/r0**2 + v_theta_0**2/r0
        
        output_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'g_eff']
        output_vals = [a0, a1, a2, c1, c2, g_eff]
        for i, name in enumerate(output_names):
            val = output_vals[i]
            outputs[name] = val