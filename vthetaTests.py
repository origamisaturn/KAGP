import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from cherryIntMDAO import OuterLoopRadialControl
from perfectTests import PerfectPitchQuery, apollo_ascent
from cherryIntLogReader import get_v_theta, get_radius, get_r_dot
from scipy.integrate import solve_ivp

def get_v_theta_dot(log):
# Gets the v_theta_dot from the state values
    v_theta = get_v_theta(log)
    t = log['state']['t']
    v_theta_dot = np.gradient(v_theta, t)
    return v_theta_dot

class VThetaTestGroup(om.Group):
    def setup(self):
        self.add_subsystem('radial_control', OuterLoopRadialControl(),
                           promotes=['*'])     
        self.add_subsystem('pitch_query', PerfectPitchQuery(), 
                           promotes=['*']) 
        self.add_subsystem('v_theta_test', VThetaTest(), 
                           promotes=['*'])

class VThetaTest(om.ExplicitComponent):
    def setup(self):
        # sample_t is associated with outer loop radial control, and must
        # correspond to the a0 and the c1 and stuff.
        # t_state is purely to obtain the current v_theta. Vary this and other states
        # while keeping sample_t constant
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        self.add_input('x_state', val=np.zeros((2)))
        self.add_input('v_state', val=np.zeros((2)))
        input_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'sample_t', 'T', 't_state',
                       'v_e', 'm0', 'm_dot', 'mu']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['a_thrust_calc', 'alpha_calc', 'r_dot_calc',
                        'r_calc', 'v_theta_calc', 'v_theta_dot_calc',
                        'v_theta_T', 'v_theta_loss_T']
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
        #g_eff = inputs['g_eff'][0]
        tau = m0 / mdot

        r0 = np.linalg.norm(pos, axis=0)
        r_hat_0 = pos/r0
        r_dot_0 = np.dot(vel, r_hat_0)


        rot_mat = np.array([[0, -1], [1, 0]])
        theta_hat_0 = rot_mat@r_hat_0
        v_theta_0 = np.dot(vel, theta_hat_0)

        state_time = inputs['t_state'][0]
        state_pos = inputs['x_state']
        state_vel = inputs['v_state']

        def get_v_theta(state_pos, state_vel):
            r = np.linalg.norm(state_pos, axis=0)
            r_hat = state_pos/r
            rot_mat = np.array([[0, -1], [1, 0]])
            theta_hat = rot_mat@r_hat
            v_theta = np.dot(state_vel, theta_hat)
            return v_theta


        def get_time_dependent_vars(t, v_theta):
            # Note, this is not the f_matrix based on T_got used in
            # OuterLoopRadial
            t_rel = t - t0

            f11 = a0*t_rel + a1*t_rel**2/2 + a2*t_rel**3/3
            f21 = a0*t_rel**2/2 + a1*t_rel**3/3 + a2*t_rel**4/4
            f22 = a0*t_rel**3/3 + a1*t_rel**4/4 + a2*t_rel**5/5
            f12 = f21
            r_dot = f11*c1 + f12*c2 + r_dot_0
            r = f21*c1 + f22*c2 + (r0 + r_dot_0*t_rel)

            p1 = a0 + a1 * (T - t) + a2 * (T - t) ** 2
            p2 = p1 * (T - t)
            r_dot_dot = c1 * p1 + c2 * p2
            a_thrust = v_e / (tau - t)
            # Note: g_eff is based on time (state actually) but we will assume
            # it is static for each iteration.
            # ASSUMPTION WRONG FOR v_theta, calculate g_eff
            g_eff = -mu/r**2 + v_theta**2/r
            alpha = math.asin((r_dot_dot - g_eff) / (a_thrust))

            return a_thrust, alpha, r_dot, r

        def v_theta_dot(t, v_theta):
            v_theta = -v_theta[0]
            # butchering the sign
            a_T, alpha, r_dot, r = get_time_dependent_vars(t, v_theta)
            v_theta_dot = a_T * math.cos(alpha) - r_dot*v_theta/r
            return -v_theta_dot
        
        def v_theta_dot_loss(t, v_theta):
            v_theta = -v_theta[0]
            a_T, alpha, r_dot, r = get_time_dependent_vars(t, v_theta)
            v_theta_dot_loss = (1-math.cos(alpha))*a_T + r_dot*v_theta/r
            return -v_theta_dot_loss
        
        state_v_theta = get_v_theta(state_pos, state_vel)
        a_thrust, alpha, r_dot, r = get_time_dependent_vars(state_time, state_v_theta)
        v_theta_dot_calc = v_theta_dot(state_time, [state_v_theta])
        output_names = ['a_thrust_calc', 'alpha_calc', 'r_dot_calc',
                        'r_calc', 'v_theta_calc', 'v_theta_dot_calc']
        # V_THETA_CALC IS NOT ACTUALLY CALC
        output_vals = [a_thrust, alpha, r_dot, r, v_theta_0, v_theta_dot_calc]
        for i, name in enumerate(output_names):
            val = output_vals[i]
            outputs[name] = val
        # outputs['v_theta_loss_calc'] = res2.y[0, -1]

        # Kept this in for sanity checking with other log files.
        t_span = [t0, T]
        res1 = solve_ivp(v_theta_dot, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
        res2 = solve_ivp(v_theta_dot_loss, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
        outputs['v_theta_T'] = res1.y[0, -1]
        outputs['v_theta_loss_T'] = res2.y[0, -1]

def plot_v_theta_error(log):
    a_thrust_calc = log['outputs']['v_theta_test.a_thrust_calc'] 
    ...

def get_v_theta_test_vals(log, prob):
    #For each point in the state, get the calculated values from VThetaTest
    t = log['state']['t']
    state = log['state']

    # Initialize output_dict with empty lists
    output_names = ['a_thrust_calc', 'alpha_calc', 'r_dot_calc',
                        'r_calc', 'v_theta_calc', 'v_theta_dot_calc', 'v_theta_T']
    output_dict = dict()
    for name in output_names:
        output_dict[name] = list()

    # determine x shape
    for i in range(len(t)):
        prob['t_state'] = t[i]
        prob['x_state'] = [state['x'][i], state['y'][i]]
        prob['v_state'] = [state['vx'][i], state['vy'][i]]

        prob.run_model()

        for name in output_names:
            output_dict[name].append(prob[name][0])

    return output_dict, t

def get_state_test_vals(log):
        # Also get the relevant calculated values from the state too
    t = log['state']['t']

    output_dict = dict()

    output_dict['r_dot'] = get_r_dot(log)
    output_dict['r'] = get_radius(log)
    output_dict['v_theta'] = get_v_theta(log)
    output_dict['v_theta_dot'] = get_v_theta_dot(log)

    return output_dict, t

def get_v_theta_test_errors(log):
    #For each point in the state, get the error
    ...

if __name__ == '__main__':
    model = VThetaTestGroup()
    log_name = "log_vtheta_test.pkl"
    prob, log = apollo_ascent(model=model, log_file=log_name)
    # Then put the log and the model into a function and feed direct
    # x, v, t values into vthetatest and compare it to the log state 
    # values
    # Values copied from apollo ascent. This whole thing is very brittle.
    r0 = 1737.4e3
    x0 = np.array([r0, 0])
    v0 = np.array([0, 2.34]) # Add rotation of moon.
    prob['x'] = x0
    prob['v'] = v0
    prob['sample_t'] = 0

    v_theta_test_vals, t = get_v_theta_test_vals(log, prob)
    state_test_vals, t = get_state_test_vals(log)

    print(v_theta_test_vals)