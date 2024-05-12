import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

def rot_mat_2d(theta):
    cos = math.cos
    sin = math.sin
    rot_mat = np.array([[cos(theta), -sin(theta)],
        [sin(theta), cos(theta)]])
    return rot_mat

def angle_between_vectors(v1, v2):
    cos_theta = np.dot(v1, v2)/(np.linalg.norm(v1) * np.linalg.norm(v2))
    return math.acos(cos_theta)

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

# WARNING This is called during time-to-go optimization, this should
# only be called in real time.
class EnginePropertyEstimator(om.ExplicitComponent):
    """ Estimates values of engine properties in-flight. 
    
    Inputs:
        sample_thrust_acceleration: [m/s^2]
        sample_t: [s] 
        estimator_ignore_time: [s] Time until which to ignore 
            sample_thrust_acceleration measurements.
        estimator_output_time: [s] Time when block will start output.
        m0: [kg]
    Outputs:
        v_e: [m/s] Estimated effective exhaust velocity of thruster.
        m_dot: [kg/s] Estimated thruster mass flow.
    
    """
    # def __init__(self, *args, **kwargs):
    #     super().__init__(*args, **kwargs)

    #     ...
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

    def compute(self, inputs, outputs):
        estimator_ignore_time = inputs['estimator_ignore_time']
        estimator_output_time = inputs['estimator_output_time']
        sample_t = inputs['sample_t']

        if sample_t < estimator_ignore_time:
            return
        else:
            sample_t = inputs['sample_t'][0]
            sample_thrust_acc = inputs['sample_thrust_acceleration'][0]
            m0 = inputs['m0'][0]

            self.thrust_acc_history.append(sample_thrust_acc)
            self.time_history.append(sample_t)

            if sample_t > 400:
                print("here")
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


class RadialControl(om.ExplicitComponent):
    """
    Component containing radial rate control block.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(x) == 2) 2D position in inertial frame. Origin is 
            at gravitational body center.
        sample_v: [m/s] (len(v) == 2) 2D velocity in inertial frame.
        sample_t: [s] Time at which sample_x and sample_v were collected.

        --- Targeting ---
        target_r_dot_T: [m/s] Target final radial velocity.
        target_r_T: [m] Target final radial position.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch.

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.


    Outputs:
        --- Component Connections ---
        a0, a1, a2, c1, c2: Coefficients used to describe radial
            acceleration over time.
        g_eff: [m/s^2] Effective gravity at time sample_t.

    Note that t = 0 is assumed to be the start time of ascent.
    """

    def setup(self):
        # 2D state.
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['sample_t', 'target_r_dot_T',
                        'target_r_T', 'T', 'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)

        output_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'g_eff']
        for name in output_names:
            self.add_output(name, val=0.0)

    def setup_partials(self):
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        # This assumes thrust on since t = 0
        
        x0 = np.array(inputs['sample_x'])
        v0 = np.array(inputs['sample_v'])
        t = inputs['sample_t'][0]
        T = inputs['T'][0]
        mu = inputs['mu'][0]
        v_e = inputs['v_e'][0]
        m_dot = inputs['m_dot'][0]
        m0 = inputs['m0'][0]
        tau = m0/m_dot
        r_dot_T = inputs['target_r_dot_T'][0]
        r_T = inputs['target_r_T'][0]
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


def get_p_coefficents(T, m0, mdot, v_e):
    """ Coefficients of p polynomial.

    Based on the Taylor series of thrust acceleration about terminal 
    time T. Derived in Appendix A.
    
    Args:
        T: [s] Terminal time; main engine cut-off.
        m0: [kg] Total mass of spacecraft at t=0.
        mdot: [kg/s] Thruster mass flow.
        v_e: [m/s] Effective exhaust velocity of thruster.

    Returns:
        a0, a1, a2 coefficients. Forms following equation:

            p1(t) = a0 + a1*(T-t) + a2(T-t)**2

    """
    tau = m0/mdot
    a0 = v_e/(tau - T)
    a1 = -a0**2/v_e
    a2 = a0**3/v_e**2
    return a0, a1, a2

def get_F_mat(t, T, a0, a1, a2):
    """ Matrix of integrals of p1 and p2 polynomials.
    
    Column 0 and 1 are p1 and p2 polynomials, while row 0 and 1 are 
    first and second integral with respect to time. Bounds are t to T.
    Described by equations (71) to (73).
    
    Args:
        t: [s]
        T: [s]
        a0, a1, a2: Coefficients from get_p_coefficients().

    Returns:
        2x2 matrix.

    """
    T_go = T - t

    f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
    f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
    f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
    f12 = f21

    F_mat = np.array([[f11, f12], [f21, f22]])
    return F_mat

def get_guidance_coefficients(t, T, F_mat, q0, q_dot_0, q_T, q_dot_T):
    """ Get coefficients for guidance equation.

    Described by equation (74). An approximation of a calculus of 
    variations solution, derived in Appendix A.

    q is generalized distance coordinate. For radial guidance it is
    radius, for yaw guidance it is normal distance from desired 
    orbital plane.
    
    Args:
        t: [s] Time at which q0 and q_dot_0 were measured.
        T: [s] Terminal time; main engine cut-off.
        F_mat: 2x2 matrix from get_F_mat().
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


class YawControl(om.ExplicitComponent):
    """ Solves equation for yaw scheduling. 
    
    Inputs:
        --- User Input ---
        --- Constants ---
        --- Component Connections ---

    Outputs:
        --- User Output ---
        --- DEBUG ---
    
    Raises: 

    """

    """ TODO: This block will receive:
        1) Desired LAN and inclination at terminal time T.
        
        This block will compute:
        1) The normal unit vector of the desired orbital plane.
        2) The out-of-plane acceleration schedule using same method as
           radial control. Output does not need transformation.

        This block will return:
        1) Components to out-of-plane acceleration schedule equation.

    """
    ...

class PitchHeadingQuery(om.ExplicitComponent):
    """ Solves for pitch and heading based on radial and out-of-plane 
    acceleration schedule. 
    
    Inputs:
        --- User Input ---
        --- Constants ---
        --- Component Connections ---

    Outputs:
        --- User Output ---
        --- DEBUG ---
    
    Raises: 

    """

    """ TODO: This block will receive:
        1) Components to out-of-plane acceleration schedule equation.
        2) Components to radial acceleration schedule equation.
        3) Desired LAN and inclination at terminal time T.
        4) Query time t.
        5) Query position x (for gravitational calculation only).

        This block will compute:
        1) Components of a_T in plane control frame (PCF)
            a. radial control component
            b. calculate k component of a_T
            c. calculate j component based on acceleration left.
            ERROR: insufficient thrust for j component/ imaginary
        2) Translate from PCF to global, then global to RCN.
        3) Calculate pitch and heading.
    """
    def setup(self):
        ...

    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        ...


class PitchQuery(om.ExplicitComponent):
    """ Component containing the equation for pitch of the fixed-thrust control
    law. Accepts equation terms from the radialControl block. This is
    called at a much higher rate than radialControl, which is called whenver 
    we want to update Tgo (at approximately 1hz).

    Inputs:
        --- User Input ---
        t: [s] Note this is NOT equal to sample_t. This is the time
            input to the equation described by the coefficients 
            given from RadialControl(). The output is alpha based on
            predicted state at time t.

        --- Constants ---
        m0: [kg] Total mass of spacecraft at launch.
        m_dot: [kg/s] Thruster mass flow.
        v_e: [m/s] Effective exhaust velocity of thruster.

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2, c1, c2: Coefficients used to describe radial
            acceleration over time.
        g_eff: [m/s^2] Effective gravity at time sample_t.

        --- DEBUG --- [CONSIDER OWN COMPONENT]
        target_r_dot_T:
        target_r_T:
    
    Outputs:
        --- User Output ---
        alpha: [rad.] Commanded pitch angle above local horizon at time
            t.

        --- DEBUG ---
        _debug: Dictionary of outputs unnecessary for guidance, but
            helpful for debugging.
            a_thrust: [m/s^2] Predicted thrust at time t.
            r_dot_dot: [m/s^2] Predicted radial acceleration at time t.
            [CONSIDER OWN COMPONENT] r_dot: [m/s] Predicted radial velocity at time t.
            [CONSIDER OWN COMPONENT] r: [m] Predicted radial position at time t.
            p1, p2: Polynomials, part of equation described by 
                RadialControl().

    Raises:
        ValueError: Error when calculating alpha. Likely indication of
            insufficient thrust or a terminal time T that is too early.

    Note that t = 0 is assumed to be the start time of ascent.
    """
    # TODO: add more descriptive value error.
    def setup(self):
        # iron out differences in t and sample_x, this is a hack.
        input_names = ['t', 'T', 'a0', 'a1', 'a2', 'c1', 'c2', 
                       'g_eff', 'm0', 'm_dot', 'v_e', 'mu',
                       'target_r_T', 'target_r_dot_T']
        for name in input_names:
            self.add_input(name, val=0.0)
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        
        output_names = ['alpha']
        for name in output_names:
            self.add_output(name, val=0.0)
        
        debug_names = ['a_thrust', 'r_dot_dot', 'p1', 'p2', 'r_dot', 'r']
        debug_dict = {}
        for name in debug_names:
            debug_dict[name] = 0.0

        self.add_discrete_output('_debug', debug_dict)

    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        t = inputs['t'][0]
        T = inputs['T'][0]
        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1 = inputs['c1'][0]
        c2 = inputs['c2'][0]
        # g_eff = inputs['g_eff'][0]
        m0 = inputs['m0'][0]
        mdot = inputs['m_dot'][0]
        mu = inputs['mu'][0]
        v_e = inputs['v_e'][0]
        tau = m0/mdot

        # here g_eff is calculated once, so it is an approximation.
        # stop out-of-bounds calculations silently
        if t > T:
            t = T
        
        p1 = a0 + a1*(T-t) + a2*(T-t)**2
        p2 = p1 * (T-t)
        r_dot_dot = c1 * p1 + c2 * p2
        a_thrust = v_e/(tau - t)

        x0 = np.array(inputs['sample_x'])
        v0 = np.array(inputs['sample_v'])
        r_dot_0, v_theta_0 = calc_2D_local_velocity(x0, v0)
        r0 = np.linalg.norm(x0)
        g_eff = -mu/r0**2 + v_theta_0**2/r0
        # Note: g_eff is based on time (state actually) but we will assume
        # it is static for each iteration.
        alpha = math.asin((r_dot_dot - g_eff)/(a_thrust))

        outputs['alpha'] = alpha

        discrete_outputs['_debug']['a_thrust'] = a_thrust
        discrete_outputs['_debug']['r_dot_dot'] = r_dot_dot
        discrete_outputs['_debug']['p1'] = p1
        discrete_outputs['_debug']['p2'] = p2

        # Copied from VThetaSolver
        r_dot_T = inputs['target_r_dot_T'][0]
        r_T = inputs['target_r_T'][0]

        F_mat = calculate_F_mat(t, T, v_e, mdot, m0)
        f11 = F_mat[0, 0]
        f12 = F_mat[0, 1]
        f21 = F_mat[1, 0]
        f22 = F_mat[1, 1]
        T_go = T - t
        r_dot = r_dot_T - f11*c1 - f12*c2
        r = r_T - (f21*c1 + f22*c2 + r_dot*T_go)

        discrete_outputs['_debug']['r_dot'] = r_dot
        discrete_outputs['_debug']['r'] = r

def calculate_F_mat(t, T, v_e, m_dot, m0):
    T_go = T - t
    tau = m0 / m_dot
    a0 = v_e/(tau - T)
    a1 = -a0**2/v_e
    a2 = a0**3/v_e**2

    f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
    f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
    f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
    f12 = f21

    F_mat = np.array([[f11, f12], [f21, f22]])
    return F_mat

class VThetaSolver(om.ExplicitComponent):
    """ Estimates final tangential velocity for given radial rate control path.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(sample_x) == 2) 2D position in inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(sample_v) == 2) 2D velocity in inertial frame.
        sample_t: [s] Time at which sample_x and sample_v were 
            collected.

        --- Targeting ---
        target_r_dot_T: [m/s] Target final radial velocity.
        target_r_T: [m] Target final radial position.

        --- Component Connections ---
        T: [s] Terminal time; main engine cut-off.
        a0, a1, a2, c1, c2: Coefficients used to describe radial
            acceleration over time.

        --- Constants ---
        mu: [m^3/s^2] Gravitational parameter.
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch.

    Outputs:
        --- Component Connections ---
        v_theta_T: [m/s] Estimated tangential velocity at terminal time
            T given radial rate control path.
        v_theta_loss_T: [m/s] Estimated potential tangential velocity
            lost to vertical thrusting by time T.


    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['sample_t', 'target_r_dot_T', 'target_r_T', 'T', 
                       'a0', 'a1', 'a2', 'c1', 'c2',  
                       'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['v_theta_T', 'v_theta_loss_T']
        for name in output_names:
            self.add_output(name, val=0.0)

    def compute(self, inputs, outputs):
        pos = inputs['sample_x']
        vel = inputs['sample_v']

        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1 = inputs['c1'][0]
        c2 = inputs['c2'][0]
        t0 = inputs['sample_t'][0]
        T = inputs['T'][0]
        m0 = inputs['m0'][0]
        mdot = inputs['m_dot'][0]
        v_e = inputs['v_e'][0]
        mu = inputs['mu'][0]
        r_dot_T = inputs['target_r_dot_T'][0]
        r_T = inputs['target_r_T'][0]
        tau = m0 / mdot

        r_dot_0, v_theta_0 = calc_2D_local_velocity(pos, vel)

        # Consider moving these to be object functions or private
        # module functions
        def get_time_dependent_vars(t, v_theta):
            # prevent out-of-bounds silently
            if t > T:
                t = T
            T_go = T - t

            f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
            f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
            f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
            f12 = f21

            r_dot = r_dot_T - f11*c1 - f12*c2
            r = r_T - (f21*c1 + f22*c2 + r_dot*T_go)

            p1 = a0 + a1 * (T_go) + a2 * (T_go) ** 2
            p2 = p1 * (T_go)
            r_dot_dot = c1 * p1 + c2 * p2
            a_thrust = v_e / (tau - t)
            g_eff = -mu/r**2 + v_theta**2/r

            alpha = math.asin((r_dot_dot - g_eff) / (a_thrust))

            return a_thrust, alpha, r_dot, r

        def v_theta_dot(t, v_theta):
            v_theta = v_theta[0]
            # butchering the sign
            a_T, alpha, r_dot, r = get_time_dependent_vars(t, v_theta)
            v_theta_dot = a_T * math.cos(alpha) - r_dot*v_theta/r
            return v_theta_dot
        
        def v_theta_dot_loss(t, v_theta):
            v_theta = v_theta[0]
            a_T, alpha, r_dot, r = get_time_dependent_vars(t, v_theta)
            v_theta_dot_loss = (1-math.cos(alpha))*a_T + r_dot*v_theta/r
            return v_theta_dot_loss
        
        t_span = [t0, T]
        try:
            T_go = T - t0
            # res1 = solve_ivp(v_theta_dot, t_span, [v_theta_0], max_step=1, atol=1, rtol=1)
            if T_go < 10:
                res1 = solve_ivp(v_theta_dot, t_span, [v_theta_0], max_step=1, atol=1, rtol=1)
            else: 
                res1 = solve_ivp(v_theta_dot, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
            #res2 = solve_ivp(v_theta_dot_loss, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
            outputs['v_theta_T'] = res1.y[0, -1]

            v_theta_T_calc = res1.y[0, -1]
            #outputs['v_theta_loss_T'] = res2.y[0, -1]
            expected_v_theta_loss_T_calc = -v_e*math.log(1 - T_go/(tau-t0)) - (v_theta_T_calc - v_theta_0)
            outputs['v_theta_loss_T'] = expected_v_theta_loss_T_calc
        except:
            print("V_theta block failed to calculate.")



class TimeToGo(om.ExplicitComponent):
    """ Generates new estimate of terminal time using fixed-point iteration.

    Inputs:
        --- User Input ---
        sample_x: [m] (len(sample_x) == 2) 2D position in inertial 
            frame. Origin is at gravitational body center.
        sample_v: [m/s] (len(sample_v) == 2) 2D velocity in inertial frame.
        sample_t: [s] Time at which sample_x and sample_v were 
            collected.
        
        --- Targeting ---
        target_v_theta_T: [m/s] Target final tangential velocity.

        --- Constants ---
        v_e: [m/s] Effective exhaust velocity of thruster.
        m_dot: [kg/s] Thruster mass flow.
        m0: [kg] Total mass of spacecraft at launch.

        --- Component Connections ---
        v_theta_T: [m/s] Estimated tangential velocity at terminal time
            T given radial rate control path.
        v_theta_loss_T: [m/s] Estimated potential tangential velocity
            lost to vertical thrusting by time T.

    Outputs:
        --- Component Connections ---
        T: [s] New predicted terminal time; main engine cut-off.

    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['sample_t', 'target_v_theta_T', 
                       'v_theta_loss_T', 'v_theta_T', 
                       'v_e', 'm0', 'm_dot']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['T', 'T_est']
        for name in output_names:
            self.add_output(name, val=0.0)

    def compute(self, inputs, outputs):
        v_theta_loss_Tn = inputs['v_theta_loss_T']
        v_theta_Tn = inputs['v_theta_T']
        target_v_theta_T = inputs['target_v_theta_T']
        t0 = inputs['sample_t']
        v_e = inputs['v_e']
        m0 = inputs['m0']
        m_dot = inputs['m_dot']

        pos = inputs['sample_x']
        vel = inputs['sample_v']

        r_dot_0, v_theta_0 = calc_2D_local_velocity(pos, vel)

        tau = m0/m_dot
        tau0 = tau - t0
        v_theta_loss_Tn_1 = target_v_theta_T - v_theta_Tn + v_theta_loss_Tn
        # Normally not negative but I have done unfortunate things with the
        # sign of v_theta
        #CHANGED V_THETA_LOSS TO TN
        T_go_n_1 = tau0 * (1 - math.exp(-(target_v_theta_T - v_theta_0 + v_theta_loss_Tn)/v_e))
        T_n_1 = T_go_n_1 + t0
        T_go_est = tau0 * (1 - math.exp(-(v_theta_Tn - v_theta_0 + v_theta_loss_Tn)/v_e))
        T_est = T_go_est + t0
        outputs['T'] = T_n_1
        outputs['T_est'] = T_est


class TimeToGo2(om.ExplicitComponent):
    """

    Inputs:


    Outputs:

    """
    def setup(self):
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['sample_t', 'target_v_theta_T', 
                       'v_theta_T', 
                       'v_e', 'm0', 'm_dot']
        for name in input_names:
            self.add_input(name, val=0.0)
        # self.add_input('Q_n', val=1.0)
        
        output_names = ['T']
        for name in output_names:
            self.add_output(name, val=0.0)
        self.add_output('Q_n', val=1.0)

        self.Q_n = 1
        self.is_first_entry = True

    def compute(self, inputs, outputs):
        Q_n = self.Q_n
        target_v_theta_T = inputs['target_v_theta_T'][0]
        v_theta_Fn = inputs['v_theta_T'][0]
        t0 = inputs['sample_t'][0]
        v_e = inputs['v_e'][0]
        m0 = inputs['m0'][0]
        m_dot = inputs['m_dot'][0]

        pos = inputs['sample_x']
        vel = inputs['sample_v']
        r_dot_0, v_theta_0 = calc_2D_local_velocity(pos, vel)

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

        self.Q_n = Q_n_1
        outputs['Q_n'] = Q_n_1
        outputs['T'] = T_n_1

class OuterLoopGroup(om.Group):
    def setup(self):
        self.add_subsystem('time_to_go', TimeToGo2(), promotes=['*'])
        self.add_subsystem('radial_control', RadialControl(),
                            promotes=['*'])
        self.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options['maxiter'] = 100
        self.nonlinear_solver.options['atol'] = 1e-3


class OuterLoopComponent(om.ExplicitComponent):
    def setup(self):
        model = OuterLoopGroup()
        self.prob = om.Problem(model)
        self.prob.setup()

        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['sample_t', 
                       'target_r_dot_T', 'target_r_T', 'target_v_theta_T',
                       'mu', 'v_e', 'm_dot', 'm0']
                       #'T'
        for name in input_names:
            self.add_input(name, val=0.0)

        # consider making this a class attribute
        output_names = ['a0', 'a1', 'a2', 'c1', 'c2',
                        'g_eff',
                        'v_theta_T',
                        'T']
        for name in output_names:
            self.add_output(name, val=0.0)

        self.add_discrete_input("run_outer_loop", val=True)
        
    def compute(self, inputs, outputs, discrete_inputs, discrete_outputs):
        self.pass_prob_inputs(inputs)
        if discrete_inputs["run_outer_loop"]:
            self.prob.run_model()
        self.pass_prob_outputs(outputs)

    def pass_prob_inputs(self, inputs):
        self.prob['sample_x'] = inputs['sample_x']
        self.prob['sample_v'] = inputs['sample_v']
        input_names = ['sample_t', 
                       'target_r_dot_T', 'target_r_T', 'target_v_theta_T',
                       'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.prob[name] = inputs[name][0]

    def pass_prob_outputs(self, outputs):
        output_names = ['a0', 'a1', 'a2', 'c1', 'c2',
                        'g_eff',
                        'v_theta_T',
                        'T']
        for name in output_names:
            outputs[name] = self.prob[name]