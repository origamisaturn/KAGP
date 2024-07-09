import numpy as np
import math
import openmdao.api as om
from scipy.optimize import least_squares

from gcherry.transform import (
    unit_vector,
    perifocal2global_rot,
    global2topo_rot,
    pcf2global_rot,
    get_ra_decl
)
from gcherry.rk4 import rk4
from gcherry.transform import global2perifocal_rot
from gcherry.log_utils import almost_equal

# NOTE: Any reference to equation numbers is in reference to 
#   "A general, explicit, optimizing guidance law for rocket-propelled spaceflight"
#   By George W. Gcherry
#


class EnginePropertyEstimator(om.ExplicitComponent):
    """ Estimates values of engine properties in-flight. 
    
    Inputs:
        --- User Input ---
        enable_estimator: (bool) If false, estimator will not output v_e 
            or m_dot values.
        sample_thrust_acceleration: [m/s^2]
        sample_t: [s] Time at which sample_thrust_acceleration, 
            sample_x, and sample_v were collected.
        estimator_ignore_time: [s] Will ignore 
            sample_thrust_acceleration measurements until 
            estimator_ignore_time seconds have occurred.
        estimator_output_time: [s] Time when block will start output.

        --- Constants ---
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

    Outputs:
        --- Component Connections ---
        v_e: [m/s] Estimated effective exhaust velocity of thruster.
        m_dot: [kg/s] Estimated thruster mass flow.
    
    """
    def minimize_function(self, x):
        v_e = x[0]
        tau = x[1]
        t = np.array(self.time_history)
        a_thrust = self.thrust_acc_history

        calc_a_thrust = v_e/(tau - t)
        res_a_thrust = calc_a_thrust - a_thrust
        
        return res_a_thrust

    def setup(self):
        self.thrust_acc_history = []
        self.time_history = []

        input_names = ['sample_thrust_acceleration', 'sample_t', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)
        self.add_input('estimator_ignore_time', val=5.0)
        self.add_input('estimator_output_time', val=100)

        output_names = ['v_e', 'm_dot']
        for name in output_names:
            self.add_output(name, val=0.0)

        self.add_discrete_input('enable_estimator', val=True)

    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        enable_estimator = discrete_inputs['enable_estimator']
        estimator_ignore_time = inputs['estimator_ignore_time']
        estimator_output_time = inputs['estimator_output_time']
        sample_t = inputs['sample_t']

        if enable_estimator:
            if sample_t < estimator_ignore_time:
                return
            else:
                sample_t = inputs['sample_t'][0]
                sample_thrust_acc = inputs['sample_thrust_acceleration'][0]
                m0 = inputs['m0'][0]

                self.thrust_acc_history.append(sample_thrust_acc)
                self.time_history.append(sample_t)

                if sample_t > estimator_output_time:
                    v_e_guess = outputs['v_e'][0]
                    m_dot_guess = outputs['m_dot'][0]
                    tau_guess = m0/m_dot_guess

                    x0 = [v_e_guess, tau_guess]
                    # Hardcoded bounds
                    bounds = ([10, 1/2], [10000, m0/0.1])
                    res = least_squares(self.minimize_function, x0, method='trf', bounds=bounds)
                    estimated_v_e = res.x[0]
                    estimated_tau = res.x[1]
                    estimated_m_dot = m0/estimated_tau

                    outputs['v_e'] = estimated_v_e
                    outputs['m_dot'] = estimated_m_dot 

class RadialYawGuidance(om.ExplicitComponent):
    """ Solves equation for pitch and yaw scheduling. 

    Note that guidance assumes a start time at t=0.
    
    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(v) == 3) 3D velocity in global inertial 
            frame.
        sample_t: [s] Time at which sample_x and sample_v were collected.

        --- Targeting ---
        target_r_T: [m] Target final radial position.
        target_r_dot_T: [m/s] Target final radial velocity.
        target_lan: [rad.] Target longitude of ascending node.
        target_inc: [rad.] Target inclination.

        --- Constants ---
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off

    Outputs:
        --- Component Connections ---
        a0, a1, a2: Coefficients of p equation.
        c1_radial, c2_radial: Coefficients for radial guidance equation.
        c1_yaw, c2_yaw: Coefficients for yaw guidance equation.
    
    """

    def setup(self):
        self.add_input('sample_x', val=np.zeros((3)))
        self.add_input('sample_v', val=np.zeros((3)))
        input_names = ['sample_t', 
                       'target_r_T', 'target_r_dot_T',
                       'target_lan', 'target_inc',
                       'v_e', 'm_dot', 'm0',
                       'T']
        for name in input_names:
            self.add_input(name, val=0.0)

        output_names = ['a0', 'a1', 'a2',
                        'c1_radial', 'c2_radial',
                        'c1_yaw', 'c2_yaw']
        for name in output_names:
            self.add_output(name, val=0.0)

    def compute(self, inputs, outputs):
        x0 = inputs['sample_x']
        v0 = inputs['sample_v']
        t = inputs['sample_t'][0]
        target_lan = inputs['target_lan'][0]
        target_inc = inputs['target_inc'][0]
        v_e = inputs['v_e'][0]
        m_dot = inputs['m_dot'][0]
        m0 = inputs['m0'][0]
        T = inputs['T'][0]

        a0, a1, a2 = _get_p_coefficients(T, m0, m_dot, v_e)
        F_mat = _get_F_mat(t, T, a0, a1, a2)
        r_hat = unit_vector(x0)

        r0 = np.linalg.norm(x0)
        r_dot_0 = np.dot(v0, r_hat)
        r_T = inputs['target_r_T'][0]
        r_dot_T = inputs['target_r_dot_T'][0]
        c1_radial, c2_radial = _get_guidance_coefficients(t, T, F_mat, r0, r_dot_0, r_T, r_dot_T)

        target_normal_vec = (perifocal2global_rot(target_lan, target_inc, 0) @ 
                            np.array([0, 0, 1]))
        # y is normal distance from target orbital plane
        y0 = np.dot(x0, target_normal_vec)
        y_dot_0 = np.dot(v0, target_normal_vec)
        y_T = 0
        y_dot_T = 0
        c1_yaw, c2_yaw = _get_guidance_coefficients(t, T, F_mat, y0, y_dot_0, y_T, y_dot_T)

        outputs['a0'] = a0
        outputs['a1'] = a1
        outputs['a2'] = a2
        outputs['c1_radial'] = c1_radial
        outputs['c2_radial'] = c2_radial
        outputs['c1_yaw'] = c1_yaw
        outputs['c2_yaw'] = c2_yaw

def _get_p_coefficients(T, m0, mdot, v_e):
    """ Coefficients of p polynomial.

    Based on the Taylor series of thrust acceleration about terminal 
    time T. Derived in Appendix A.
    
    Args:
        T: [s] Terminal time; main engine cut-off.
        m0: [kg] Total mass of spacecraft at t=0.
        mdot: [kg/s] Thruster mass flow.
        v_e: [m/s] Effective exhaust velocity of thruster.

    Returns:
        a0, a1, a2 coefficients. Forms following equations:

            p1(t) = a0 + a1*(T-t) + a2(T-t)**2
            p2(t) = p1(t)*(T-t)

    """
    tau = m0/mdot
    a0 = v_e/(tau - T)
    a1 = -a0**2/v_e
    a2 = a0**3/v_e**2
    return a0, a1, a2

def _get_F_mat(t, T, a0, a1, a2):
    """ Matrix of integrals of p1 and p2 polynomials.
    
    Column 0 and 1 are p1 and p2 polynomials, while row 0 and 1 are 
    first and second integral with respect to time. Bounds are t to T.
    Described by equations (71) to (73).
    
    Args:
        t: [s] Time at which q0 and q_dot_0 were measured.
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2: Coefficients from _get_p_coefficients().

    Returns:
        2x2 array[float].

    """
    T_go = T - t

    f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
    f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
    f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
    f12 = f21

    F_mat = np.array([[f11, f12], [f21, f22]])
    return F_mat

def _get_guidance_coefficients(t, T, F_mat, q0, q_dot_0, q_T, q_dot_T):
    """ Get coefficients for guidance equation.

    Described by equation (74). An approximation of a calculus of 
    variations solution, derived in Appendix A.

    q is generalized distance coordinate. For radial guidance it is
    radius, for yaw guidance it is normal distance from desired 
    orbital plane.
    
    Args:
        t: [s] Time at which q0 and q_dot_0 were measured.
        T: [s] Terminal time; main engine cut-off.
        F_mat: [2x2 array[float]] from _get_F_mat(). Should have equal 
            t and T arguments as this function.
        q0: [m] Initial boundary condition, distance.
        q_dot_0: [m/s] Initial boundary condition, velocity.
        q_T: [m] Final boundary condition, distance.
        q_dot_T: [m/s] Final boundary condition, velocity.

    Returns:
        c1 and c2 coefficients. Forms following equation:

            q_dot_dot(t) = c1*p1(t) + c2*p2(t)

    """
    Tgo = T - t
    f11 = F_mat[0][0]
    f12 = F_mat[0][1]
    f21 = F_mat[1][0]
    f22 = F_mat[1][1]
    A = np.array([[f11, f12], [f21, f22]])
    b = np.array([[q_dot_T - q_dot_0], 
                [q_T - (q0 + q_dot_0*Tgo)]])
    c = np.linalg.solve(A, b).reshape(2)

    return c[0], c[1]

class PitchHeadingQuery(om.ExplicitComponent):
    """ Solves for pitch and heading based on radial and out-of-plane 
    acceleration schedule. 

    Note that guidance assumes a start time at t=0.
    
    Inputs:
        --- User Input ---
        query_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center. Used for
            g_eff and frame transformations.
        query_v: [m/s] [m/s] (len(v) == 3) 3D velocity in global inertial 
            frame. Used only for g_eff.
        query_t: [s] Time at which query_x and query_v occurred.

        --- Targeting ---
        target_lan: [rad.] Target longitude of ascending node.
        target_inc: [rad.] Target inclination.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2: Coefficients of p equation.
        c1_radial, c2_radial: Coefficients for radial guidance equation.
        c1_yaw, c2_yaw: Coefficients for yaw guidance equation.

        --- DEBUG ---
        target_r_T: [m] Target final radial position.
        target_r_dot_T: [m/s] Target final radial velocity.

    Outputs:
        --- User Output ---
        cmd_pitch: [rad.] Commanded pitch.
        cmd_heading: [rad.] Commanded heading.

        --- DEBUG ---
        _debug: dict containing:
            r: [m] Expected radius at query_t.
            r_dot: [m/s] Expected radial velocity at time query_t.
            r_dot_dot: [m/s**2] Expected radial acceleration at time 
                query_t.
            y: [m] Expected normal distance from target orbital plane at
                time query_t.
            y_dot: [m/s] 
            y_dot_dot: [m/s**2]

    """
    def setup(self):
        self.add_input('query_x', val=np.zeros((3)))
        self.add_input('query_v', val=np.zeros((3)))
        input_names = ['query_t',
                       'target_lan', 'target_inc',
                       'mu', 'v_e', 'm_dot', 'm0',
                       'a0', 'a1', 'a2',
                       'c1_radial', 'c2_radial',
                       'c1_yaw', 'c2_yaw',
                       'T',
                       'target_r_T', 'target_r_dot_T']
        for name in input_names:
            self.add_input(name, val=0.0)

        output_names = ['cmd_pitch', 'cmd_heading']
        for name in output_names:
            self.add_output(name, val=0.0)

        self.add_discrete_output('_debug', val={
            'r': 0,
            'r_dot': 0,
            'r_dot_dot': 0,
            'y': 0,
            'y_dot': 0,
            'y_dot_dot': 0 })

    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        x0 = inputs['query_x']
        v0 = inputs['query_v']
        t = inputs['query_t'][0]
        target_lan = inputs['target_lan'][0]
        target_inc = inputs['target_inc'][0]
        mu = inputs['mu'][0]
        v_e = inputs['v_e'][0]
        m_dot = inputs['m_dot'][0]
        m0 = inputs['m0'][0]
        T = inputs['T'][0]
        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1_radial = inputs['c1_radial'][0]
        c2_radial = inputs['c2_radial'][0]
        c1_yaw = inputs['c1_yaw'][0]
        c2_yaw = inputs['c2_yaw'][0]
        
        tau = m0/m_dot
        r_hat = unit_vector(x0)
        # Able to handle case where v_theta_0 == 0.
        v_theta_0 = (np.linalg.norm(v0)**2 - np.dot(v0, r_hat)**2)**0.5
        r0 = np.linalg.norm(x0)
        g = -mu/r0**2 * r_hat
        g_eff = -mu/r0**2 + v_theta_0**2/r0
        a_thrust_mag = v_e/(tau - t)
        p1 = a0 + a1*(T-t) + a2*(T-t)**2
        p2 = p1 * (T-t)

        # Find cmd_pitch
        r_dot_dot = c1_radial*p1 + c2_radial*p2
        a_thrust_r = r_dot_dot - g_eff
        cmd_pitch = math.asin(a_thrust_r/a_thrust_mag)

        # Find cmd_heading
        # y is component along normal of target orbital plane
        target_normal_vec = (perifocal2global_rot(target_lan, target_inc, 0) @ 
                            np.array([0, 0, 1]))
        y_dot_dot = c1_yaw*p1 + c2_yaw*p2
        a_thrust_y = y_dot_dot - np.dot(g, target_normal_vec)
        a_thrust_global = _guidance_to_global(a_thrust_r, a_thrust_y, a_thrust_mag, x0, target_lan, target_inc)
        a_thrust_topo = global2topo_rot(*(get_ra_decl(x0)))@a_thrust_global
        # heading from 0 deg to 360 deg.
        if not (almost_equal(a_thrust_topo[0], 0, 1e-8) and 
                almost_equal(a_thrust_topo[1], 0, 1e-8)):
            cmd_heading = np.arctan2(a_thrust_topo[1], a_thrust_topo[0])%(2*np.pi)
        else:
            cmd_heading = 0

        # Set outputs
        outputs["cmd_pitch"] = cmd_pitch
        outputs["cmd_heading"] = cmd_heading

        ## DEBUG OUTPUTS
        F_mat = _get_F_mat(t, T, a0, a1, a2)
        f11 = F_mat[0, 0]
        f12 = F_mat[0, 1]
        f21 = F_mat[1, 0]
        f22 = F_mat[1, 1]
        T_go = T - t

        # TODO: Turn this into a function
        # This would be _get_expected_guidance_values()
        r_dot_T = inputs['target_r_dot_T']
        r_T = inputs['target_r_T'][0]
        r_dot = r_dot_T - f11*c1_radial - f12*c2_radial
        r = r_T - (f21*c1_radial + f22*c2_radial + r_dot*T_go)
        discrete_outputs['_debug']['r'] = r
        discrete_outputs['_debug']['r_dot'] = r_dot
        discrete_outputs['_debug']['r_dot_dot'] = r_dot_dot

        y_T = 0
        y_dot_T = 0
        y_dot = y_dot_T - f11*c1_yaw - f12*c2_yaw
        y = y_T - (f21*c1_yaw + f22*c2_yaw + y_dot*T_go)
        discrete_outputs['_debug']['y'] = y
        discrete_outputs['_debug']['y_dot'] = y_dot
        discrete_outputs['_debug']['y_dot_dot'] = y_dot_dot

        discrete_outputs['_debug']['a_thrust_mag'] = a_thrust_mag

def _guidance_to_global(a_thrust_r, a_thrust_y, a_thrust_mag, pos_global, target_lan, target_inc):
    """ Converts guidance commands to global thrust vector. 

    Args:
        a_thrust_r: [m/s**2] Commanded radial thrust.
        a_thrust_y: [m/s**2] Commanded target orbit normal thrust.
        a_thrust_mag: [m/s**2] Current thrust acceleration.
        pos_global: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.
        target_lan: [rad.] Target longitude of ascending node.
        target_inc: [rad.] Target inclination.
    
    Returns:
        Length 3 array of thrust acceleration [m/s**2] in global 
        frame.
    
    """
    target_normal_vec = (perifocal2global_rot(target_lan, target_inc, 0) @ 
                        np.array([0, 0, 1]))
    # _i, _j, _k are components along plane control axes.
    pcf_axes = pcf2global_rot(pos_global, target_lan, target_inc)
    a_thrust_i = a_thrust_r

    # Find k component based on _i and _y component of thrust
    target_normal_vec_pcf = pcf_axes.T@target_normal_vec
    y1_hat = target_normal_vec_pcf
    a_thrust_k = (a_thrust_y - a_thrust_r*y1_hat[0])/y1_hat[2]

    radicand = a_thrust_mag**2 - a_thrust_i**2 - a_thrust_k**2
    if radicand < 0:
        raise ValueError("Cannot take square root of {}: a_thrust_j will be imaginary.".format(radicand))
    
    a_thrust_j = np.sqrt(radicand)
    a_thrust_pcf = np.array([a_thrust_i, a_thrust_j, a_thrust_k])
    a_thrust_global = pcf2global_rot(pos_global, target_lan, target_inc)@a_thrust_pcf
    return a_thrust_global

class VThetaSolver(om.ExplicitComponent):
    """ Estimates final tangential velocity for given radial rate control path.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(v) == 3) 3D velocity in global inertial 
            frame.
        sample_t: [s] Time at which sample_x and sample_v were collected.

        --- Targeting ---
        target_r_T: [m] Target final radial position.
        target_r_dot_T: [m/s] Target final radial velocity.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2: Coefficients of p equation.
        c1_radial, c2_radial: Coefficients for radial guidance equation.
        c1_yaw, c2_yaw: Coefficients for yaw guidance equation.

    Outputs:
        --- Component Connections ---
        v_theta_T: [m/s] Estimated tangential velocity at terminal time
            T given radial rate control path.
        v_theta_loss_T: [m/s] Estimated potential tangential velocity
            lost to vertical thrusting by time T.
        delta_theta_T: [rad.] Estimated change in true anomaly from 
            sample_t to terminal time T.

    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((3)))
        self.add_input('sample_v', val=np.zeros((3)))
        input_names = ['sample_t', 
                       'target_r_T', 'target_r_dot_T',
                       'mu', 'v_e', 'm_dot', 'm0',
                       'T', 
                       'a0', 'a1', 'a2',
                       'c1_radial', 'c2_radial',
                       'c1_yaw', 'c2_yaw']
        for name in input_names:
            self.add_input(name, val=0.0)

        # self.add_input('query_t', val=0.0)
        
        output_names = ['v_theta_T', 'v_theta_loss_T', 'delta_theta_T']
        for name in output_names:
            self.add_output(name, val=0.0)

        # self.add_discrete_output('_debug', val={
        #     'v_theta_dot': 0 })

    def compute(self, inputs, outputs):
        pos = inputs['sample_x']
        vel = inputs['sample_v']
        t0 = inputs['sample_t'][0]
        target_r_T = inputs['target_r_T'][0]
        target_r_dot_T = inputs['target_r_dot_T'][0]
        mu = inputs['mu'][0]
        v_e = inputs['v_e'][0]
        m_dot = inputs['m_dot'][0]
        m0 = inputs['m0'][0]
        T = inputs['T'][0]
        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1_radial = inputs['c1_radial'][0]
        c2_radial = inputs['c2_radial'][0]
        c1_yaw = inputs['c1_yaw'][0]
        c2_yaw = inputs['c2_yaw'][0]

        tau = m0/m_dot
        r_hat = unit_vector(pos)
        v_theta_0 = (np.linalg.norm(vel)**2 - np.dot(vel, r_hat)**2)**0.5
        
        def get_state_dot(t, state):
            v_theta = state[0]
            delta_theta = state[1]

            T_go = T - t
            a_thrust_mag = v_e/(tau - t)
            F_mat = _get_F_mat(t, T, a0, a1, a2)
            f11 = F_mat[0, 0]
            f12 = F_mat[0, 1]
            f21 = F_mat[1, 0]
            f22 = F_mat[1, 1]
            p1 = a0 + a1*(T-t) + a2*(T-t)**2
            p2 = p1 * (T-t)

            r_dot = target_r_dot_T - f11*c1_radial - f12*c2_radial
            r = target_r_T - (f21*c1_radial + f22*c2_radial + r_dot*T_go)
            y_dot = 0 - f11*c1_yaw - f12*c2_yaw
            y = 0 - (f21*c1_yaw + f22*c2_yaw + y_dot*T_go)
            g_eff = -mu/r**2 + v_theta**2/r

            r_dot_dot = c1_radial*p1 + c2_radial*p2
            a_thrust_r = r_dot_dot - g_eff
            y_dot_dot = c1_yaw*p1 + c2_yaw*p2
            a_thrust_y = y_dot_dot + (mu/r**2)*(y/r)

            cbeta = np.sqrt(r**2 - y**2)/r
            sbeta = y/r
            y_unit_PCF = np.array([sbeta, 0, cbeta])

            # i, j, k are axes in Plane Control Frame
            a_thrust_i = a_thrust_r
            a_thrust_k = (a_thrust_y - a_thrust_i*y_unit_PCF[0])/y_unit_PCF[2]
            a_thrust_j = np.sqrt(a_thrust_mag**2 - a_thrust_i**2 - a_thrust_k**2)
            a_thrust_PCF = np.array([a_thrust_i, a_thrust_j, a_thrust_k])

            theta_hat_PCF = np.zeros(3)
            if v_theta != 0:
                vel_i = r_dot
                v_mag = np.sqrt(v_theta**2 + vel_i**2)
                vel_k = (y_dot - vel_i*y_unit_PCF[0])/y_unit_PCF[2]
                vel_j = np.sqrt(v_mag**2 - vel_i**2 - vel_k**2)
                theta_hat_PCF = np.array([
                    0,
                    vel_j,
                    vel_k
                    ]) / v_theta
                theta_dot = vel_j/r
            else:
                theta_hat_PCF = unit_vector(np.array([
                    0,
                    a_thrust_PCF[1],
                    a_thrust_PCF[2]
                    ]))
                theta_dot = 0

            a_thrust_theta = np.dot(a_thrust_PCF, theta_hat_PCF)
            
            v_theta_dot = a_thrust_theta - r_dot * v_theta/r
            # theta is angle on target orbital plane.
            return np.array([v_theta_dot, theta_dot])
        
        tspan = [t0, T]
        max_step = 1
        t_res, y_res = rk4(get_state_dot, tspan, [v_theta_0, 0], max_step)

        T_go = T-t0
        estimated_v_theta_T = y_res[0, -1]
        estimated_delta_theta_T = y_res[1, -1]
        # TODO: come up with more robust method of dealing with guidance
        # exceeding thrust requirements.
        if np.isnan(estimated_v_theta_T):
            estimated_v_theta_T = 0
            estimated_delta_theta_T = 0

        estimated_v_theta_loss_T = -v_e*math.log(1 - T_go/(tau-t0)) - (estimated_v_theta_T - v_theta_0)

        outputs['v_theta_T'] = estimated_v_theta_T
        outputs['v_theta_loss_T'] = estimated_v_theta_loss_T
        outputs['delta_theta_T'] = estimated_delta_theta_T

        # discrete_outputs['_debug']['v_theta_dot'] = get_v_theta_dot

class TimeToGo(om.ExplicitComponent):
    """ Fixed-point iteration for terminal time T.
    
    Creates new estimate of T after checking estimated
    v_theta_T against target v_theta_T.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(v) == 3) 3D velocity in global inertial 
            frame.
        sample_t: [s] Time at which sample_x and sample_v were collected.

        --- Targeting ---
        target_v_theta_T: [m/s] Target final circumferential speed.

        --- Constants ---
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

        --- Component Connections ---
        v_theta_T: [m/s] Estimated tangential velocity at terminal time
            T given radial rate control path.


    Outputs:
        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.

        --- DEBUG ---
        Q_n: e**(-v_theta_loss_T/v_e) (Eq. 155) Variable which changes
            to solve for new guess of T.

    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((3)))
        self.add_input('sample_v', val=np.zeros((3)))
        input_names = ['sample_t', 'target_v_theta_T', 
                       'v_theta_T', 
                       'v_e', 'm0', 'm_dot']
        for name in input_names:
            self.add_input(name, val=0.0)
        # self.add_input('Q_n', val=1.0)
        
        self.add_output('T', val=0.0)
        self.add_output('Q_n', val=1.0)

        self._Q_n = 1
        self.is_first_entry = True

    def compute(self, inputs, outputs):
        Q_n = self._Q_n
        target_v_theta_T = inputs['target_v_theta_T'][0]
        v_theta_Fn = inputs['v_theta_T'][0]
        t0 = inputs['sample_t'][0]
        v_e = inputs['v_e'][0]
        m0 = inputs['m0'][0]
        m_dot = inputs['m_dot'][0]

        pos = inputs['sample_x']
        vel = inputs['sample_v']
        r_hat = unit_vector(pos)
        v_theta_0 = (np.linalg.norm(vel)**2 - np.dot(vel, r_hat)**2)**0.5

        tau = m0/m_dot
        tau0 = tau - t0
        P = math.exp(-(target_v_theta_T - v_theta_0)/v_e)

        if not self.is_first_entry:
            H_fn = math.exp(-(v_theta_Fn - v_theta_0)/v_e)
            Q_n_1 = P * Q_n / H_fn
        else:
            Q_n_1 = Q_n
            self.is_first_entry = False

        T_go_n_1 = tau0*(1 - P*Q_n_1)
        T_n_1 = T_go_n_1 + t0

        # TODO: good metric for impossible trajectories is to use tau
        # for T, and if it doesn't run it is bad. For getting VThetaSolver
        # to output real values, there should be no upper bound on T.
        self._Q_n = Q_n_1
        outputs['Q_n'] = Q_n_1
        outputs['T'] = T_n_1

class OrbitTargeting(om.ExplicitComponent):
    """ Iterative solving for target guidance parameters.

    Estimates new guess for final radial and circumferential state based
    on calculated final true anomaly.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.

        --- Targeting ---
        target_lan: [rad.] Target longitude of ascending node.
        target_inc: [rad.] Target inclination.
        target_pe: [m] Target periapsis.
        target_ap: [m] Target apoapsis.
        target_argp: [rad.] Target argument of periapsis.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.

        --- Component Connections ---
        delta_theta_T: [rad.] Estimated change in true anomaly from 
            sample_t to terminal time T.

    Outputs:
        --- Component Connections ---
        target_r_T: [m] Target final radial position.
        target_r_dot_T: [m/s] Target final radial velocity.
        target_v_theta_T: [m/s] Target final circumferential speed.
        
    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((3)))
        input_names = ['target_lan', 'target_inc', 
                       'target_pe', 'target_ap',
                       'target_argp',
                       'mu',
                       'delta_theta_T']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['target_r_T', 'target_r_dot_T', 'target_v_theta_T']
        for name in output_names:
            self.add_output(name, val=0.0)

    def compute(self, inputs, outputs):
        x0 = inputs['sample_x']
        target_lan = inputs['target_lan'][0]
        target_inc = inputs['target_inc'][0]
        target_pe = inputs['target_pe'][0]
        target_ap = inputs['target_ap'][0]
        target_argp = inputs['target_argp'][0]
        mu = inputs['mu'][0]
        delta_theta_T = inputs['delta_theta_T'][0]

        x0_perifocal = global2perifocal_rot(target_lan, target_inc, target_argp)@x0
        sample_true_anomaly = np.arctan2(x0_perifocal[1], x0_perifocal[0])
        estimated_true_anomaly = sample_true_anomaly + delta_theta_T
        r_T, r_dot_T, v_theta_T = _orbit_to_guidance_target(
            target_pe, target_ap, estimated_true_anomaly, mu)
        
        outputs['target_r_T'] = r_T
        outputs['target_r_dot_T'] = r_dot_T
        outputs['target_v_theta_T'] = v_theta_T
      
def _orbit_to_guidance_target(periapsis, apoapsis, true_anomaly, 
                        gravitational_parameter):
    """ Convert target orbital parameters to radial guidance targets.

    Args:
        periapsis: [m]
        apoapsis: [m]
        true_anomaly: [m]
        gravitational_parameter: [m^3/s^2]

    Returns:
        r [m] radius, v_r [m/s] radial velocity, v_theta [m/s] 
            circumferential velocity. 

    """
    mu = gravitational_parameter
    r_p = periapsis
    r_a = apoapsis
    theta = true_anomaly

    cos = np.cos
    sin = np.sin

    a = 1/2 * (r_p + r_a)
    e = 1 - r_p/a
    r = a * (1 - e**2)/(1 + e*cos(theta))

    h = (r_p * mu * (1+e))**0.5
    v_r = mu/h * e * sin(theta)
    v_theta = h/r

    return r, v_r, v_theta

class OuterLoopGroup(om.Group):
    """ Group that solves for ascent given target orbital plane and final
    radial position, velocity, and circumferential velocity.

    """
    def setup(self):
        self.add_subsystem('time_to_go', TimeToGo(), promotes=['*'])
        self.add_subsystem('radial_yaw_guidance', RadialYawGuidance(),
                            promotes=['*'])
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options['maxiter'] = 100
        self.nonlinear_solver.options['atol'] = 1e-3

class OrbitTargetingGroup(om.Group):
    """ Group for ascent given target orbital parameters. 

    """
    def setup(self):
        self.add_subsystem('orbit_targeting', OrbitTargeting(), promotes=['*'])
        self.add_subsystem('outer_loop', OuterLoopGroup(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options['maxiter'] = 100
        self.nonlinear_solver.options['atol'] = 1e-3

class OrbitTargetingAndEngineEstimatorGroup(om.Group):
    """ Solves for ascent given target orbital parameters, with estimator
    for engine parameters."""
    def setup(self):
        self.add_subsystem('engine_estimator', EnginePropertyEstimator(), promotes=['*'])
        self.add_subsystem('orbit_targeting', OrbitTargetingGroup(), promotes=['*'])

class OrbitGuidanceComponent(om.ExplicitComponent):
    """ Solves for ascent guidance equation given target orbital 
    parameters.
    
    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 3) 3D position in global inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(v) == 3) 3D velocity in global inertial 
            frame.
        sample_t: [s] Time at which sample_x and sample_v were collected.

        --- User Input (Estimator) ---
        enable_estimator: (bool) If false, estimator will not output v_e 
            or m_dot values.
        sample_thrust_acceleration: [m/s^2]
        estimator_ignore_time: [s] Will ignore 
            sample_thrust_acceleration measurements until 
            estimator_ignore_time seconds have occurred.
        estimator_output_time: [s] Time when block will start output.

        --- Targeting ---
        target_lan: [rad.] Target longitude of ascending node.
        target_inc: [rad.] Target inclination.
        target_pe: [m] Target periapsis.
        target_ap: [m] Target apoapsis.
        target_argp: [rad.] Target argument of periapsis.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.
        v_e: (ESTIMATOR OUTPUT) [m/s] Effective exhaust velocity of thruster.
        m_dot: (ESTIMATOR OUTPUT) [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch (sample_t == 0).

        --- Component Connections ---
        delta_theta_T: [rad.] Estimated change in true anomaly from 
            sample_t to terminal time T.

    Outputs:
        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2: Coefficients of p equation.
        c1_radial, c2_radial: Coefficients for radial guidance equation.
        c1_yaw, c2_yaw: Coefficients for yaw guidance equation.

        --- Unused ---
        target_r_T: [m] Target final radial position.
        target_r_dot_T: [m/s] Target final radial velocity.
        target_v_theta_T: [m/s] Target final circumferential speed.
        v_theta_T: [m/s] Estimated tangential velocity at terminal time
            T given radial rate control path.
        delta_theta_T: [rad.] Estimated change in true anomaly from 
            sample_t to terminal time T.
        
    """
    def setup(self):
        model = OrbitTargetingAndEngineEstimatorGroup()
        self.prob = om.Problem(model)
        self.prob.setup()

        self.add_input('sample_x', val=np.zeros((3)))
        self.add_input('sample_v', val=np.zeros((3)))
        input_names = ['sample_t', 
                       'sample_thrust_acceleration',
                       'target_pe', 'target_ap', 'target_argp',
                       'target_lan', 'target_inc',
                       'mu', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)
        self.add_input('estimator_ignore_time', val=7.0)
        self.add_input('estimator_output_time', val=100.0)

        # consider making this a class attribute
        output_names = ['a0', 'a1', 'a2',
                        'c1_radial', 'c2_radial',
                        'c1_yaw', 'c2_yaw',
                        'v_theta_T', 'delta_theta_T',
                        'T',
                        'target_r_T', 'target_r_dot_T',
                        'target_v_theta_T',
                        'v_e', 'm_dot']
        for name in output_names:
            self.add_output(name, val=0.0)

        self.add_discrete_input("run_outer_loop", val=True)
        self.add_discrete_input("enable_estimator", val=True)
        
    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        self.pass_prob_inputs(inputs)
        if discrete_inputs["run_outer_loop"]:
            self.prob.run_model()
        self.pass_prob_outputs(outputs)

    def pass_prob_inputs(self, inputs):
        self.prob['sample_x'] = inputs['sample_x']
        self.prob['sample_v'] = inputs['sample_v']
        input_names = ['sample_t', 
                       'target_pe', 'target_ap', 'target_argp',
                       'target_lan', 'target_inc',
                       'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.prob[name] = inputs[name][0]

    def pass_prob_outputs(self, outputs):
        output_names = ['a0', 'a1', 'a2',
                        'c1_radial', 'c2_radial',
                        'c1_yaw', 'c2_yaw',
                        'v_theta_T', 'delta_theta_T',
                        'T',
                        'target_r_T', 'target_r_dot_T',
                        'target_v_theta_T']
        for name in output_names:
            outputs[name] = self.prob[name]


class VThetaSolverTestGroup(om.Group):
    """ Group for debugging guidance without iterative solver components.
    """
    def setup(self):
        self.add_subsystem('radial_yaw_guidance', RadialYawGuidance(),
                            promotes=['*'])
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])

class VThetaSolverOuterLoop(om.ExplicitComponent):
    """ Component for debugging guidance without iterative solver components.
    """
    def setup(self):
        model = VThetaSolverTestGroup()
        self.prob = om.Problem(model)
        self.prob.setup()

        self._vector_input_names = ['sample_x', 'sample_v']
        self._scalar_input_names = ['sample_t', 
                       'target_r_T', 'target_r_dot_T',
                       'target_lan', 'target_inc',
                       'mu', 'v_e', 'm_dot', 'm0',
                       'T']
        for name in self._scalar_input_names:
            self.add_input(name, val=0.0)
        for name in self._vector_input_names:
            self.add_input(name, val=np.zeros((3)))

        self._scalar_output_names = ['a0', 'a1', 'a2',
                        'c1_radial', 'c2_radial',
                        'c1_yaw', 'c2_yaw',
                        'v_theta_T', 'v_theta_loss_T', 'delta_theta_T']
        for name in self._scalar_output_names:
            self.add_output(name, val=0.0)

        self.add_discrete_input("run_outer_loop", val=True)

    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        self.pass_prob_inputs(inputs)
        if discrete_inputs["run_outer_loop"]:
            self.prob.run_model()
        self.pass_prob_outputs(outputs)

    def pass_prob_inputs(self, inputs):
        for name in self._scalar_input_names:
            self.prob[name] = inputs[name][0]
        for name in self._vector_input_names:
            self.prob[name] = inputs[name]

    def pass_prob_outputs(self, outputs):
        for name in self._scalar_output_names:
            outputs[name] = self.prob[name]


# TODO: Figure out what to do with this function.
def _get_expected_guidance_values(t, T, F_mat, c1, c2, q_T, q_dot_T):
    """ Obtain expected coordinate and its derivative at time t.

    q is generalized distance coordinate. For radial guidance it is
    radius, for yaw guidance it is normal distance from desired 
    orbital plane. Based on equations (24) and (25).
    
    Args:
        t: [s] Time at which q0 and q_dot_0 were measured.
        T: [s] Terminal time; main engine cut-off.
        F_mat: [2x2 array[float]] from _get_F_mat(). Should have equal 
            t and T arguments as this function.
        q_T: [m] Final boundary condition, distance.
        q_dot_T: [m/s] Final boundary condition, velocity.

    Returns:
        q, q_dot which are the expected values at time
        t.
    
    """
    f11 = F_mat[0, 0]
    f12 = F_mat[0, 1]
    f21 = F_mat[1, 0]
    f22 = F_mat[1, 1]
    T_go = T - t

    q_dot = q_dot_T - f11*c1 - f12*c2
    q = q_T - (f21*c1+ f22*c2 + q_dot*T_go)

    return q, q_dot
