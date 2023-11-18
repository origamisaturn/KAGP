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


class RadialControl(om.ExplicitComponent):
    """
    Component containing radial rate control block.
    """

    def setup(self):
        """
        inputs:
            position
            velocity
            T_go
            thrust i guess
            a bunch of params
        outputs:
            angle
        """
        # The way this is set up I will have to put a lot of information
        # in the comments, I don't know exactly how to organize OpenMDAO
        # stuff yet.
        self.add_input('x', val=np.zeros((2)))
        self.add_input('v', val=np.zeros((2)))
        input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'T', 'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)

        output_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'tau', 'g_eff']
        for name in output_names:
            self.add_output(name, val=0.0)

    def setup_partials(self):
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        # NEED TO DEFINE AI
        # This assumes thrust on since t = 0
        # get radial components of these, these are x0 vectors
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

        # Define initial boundary vectors
        # We are assuming planet center is at 0
        # theta_hat is 90 degrees clockwise of r_hat
        deg_to_pi = math.pi/180
        theta = -90 * deg_to_pi

        r0 = np.linalg.norm(x0)
        r_hat_0 = x0/r0
        theta_hat_0 = (rot_mat_2d(theta)@r_hat_0.reshape((2,1))).reshape(2)

        r_dot_0 = np.dot(v0.T, r_hat_0)
        v_theta_0 = np.dot(v0.T, theta_hat_0)


        # Calculate F matrix
        # Compute values of ai
        # a0 is equivalent to thrust at terminal time
        a0 = v_e/(tau - T)
        a1 = -a0**2/v_e
        a2 = a0**3/v_e**2

        f11 = a0*T_go + a1*T_go**2/2 + a2*T_go**3/3
        f21 = a0*T_go**2/2 + a1*T_go**3/3 + a2*T_go**4/4
        f22 = a0*T_go**3/3 + a1*T_go**4/4 + a2*T_go**5/5
        f12 = f21

        # Ax = b, where A is F vector, b is boundary conditions, x is 
        # [c1, c2]

        A = np.array([[f11, f12], [f21, f22]])
        b = np.array([[r_dot_T - r_dot_0], 
                    [r_T - (r0 + r_dot_0*T_go)]])
        c = np.linalg.solve(A, b).reshape(2)

        c1 = c[0]
        c2 = c[1]

        # Compute effective g_eff at current time. This is a function of t,
        # but we will assume it is static for each iteration.
        g_eff = -mu/r0**2 + v_theta_0**2/r0
        
        output_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'tau', 'g_eff']
        output_vals = [a0, a1, a2, c1, c2, tau, g_eff]
        for i, name in enumerate(output_names):
            val = output_vals[i]
            outputs[name] = val

class OuterLoopRadialControl(om.ExplicitComponent):
    # This is just the radial control block with extra logic. Drop-in
    # for when I introduce Tgo.
    # ooh I can have equal input and output function blocks if I call 
    # them from the components and substitute my self object into them.
    # Only updates itself when sample_t changes from last iteration.
    def setup(self):
        model = om.Group()
        model.add_subsystem('radial_control', RadialControl(), 
                            promotes=['*'])
        
        self.prob = om.Problem(model)
        self.prob.setup()
        # recorder = om.SqliteRecorder('cases.sql')
        # self.prob.add_recorder(recorder)

        self.add_input('x', val=np.zeros((2)))
        self.add_input('v', val=np.zeros((2)))
        # SAMPLE T SHOULD BE negative interval since t0 = 0.
        input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'T', 'mu', 'v_e', 'm_dot', 'm0']
        for name in input_names:
            self.add_input(name, val=0.0)

        self.add_output('sample_x', val=np.zeros((2)))
        self.add_output('sample_v', val=np.zeros((2)))
        output_names = ['last_sample_t', 'a0',
                        'a1', 'a2', 'c1', 'c2', 'tau', 'g_eff']
        for name in output_names:
            self.add_output(name, val=0.0)
        
        self.last_sample_t = -1

    def compute(self, inputs, outputs):
        sample_t = inputs['sample_t'][0]
        if self.last_sample_t != sample_t:
            # This should be its own function
            self.prob['x'] = inputs['x']
            self.prob['v'] = inputs['v']
            input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'T', 'mu', 'v_e', 'm_dot', 'm0']
            for name in input_names:
                self.prob[name] = inputs[name][0]

            self.prob.run_model()

            output_names = ['a0', 'a1', 'a2', 'c1',
                    'c2', 'tau', 'g_eff']
            for name in output_names:
                outputs[name] = self.prob[name]

            self.last_sample_t = sample_t
            outputs['sample_x'] = inputs['x']
            outputs['sample_v'] = inputs['v']
            outputs['last_sample_t'] = self.last_sample_t

class PitchQuery(om.ExplicitComponent):
    """
    Component containing the equation for pitch of the fixed-thrust control
    law. Accepts equation terms from the radialControl block. This is
    called at a much higher rate than radialControl, which is called whenver 
    we want to update Tgo (at approximately 1hz).
    """

    def setup(self):
        input_names = ['t', 'T', 'a0', 'a1', 'a2', 'c1', 'c2', 
                       'g_eff', 'm0', 'm_dot', 'v_e']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['alpha', 'a_thrust', 'r_dot_dot', 'p1', 'p2']
        for name in output_names:
            self.add_output(name, val=0.0)

    # Did not add setup_partials
    def compute(self, inputs, outputs):
        t = inputs['t'][0]
        T = inputs['T'][0]
        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1 = inputs['c1'][0]
        c2 = inputs['c2'][0]
        g_eff = inputs['g_eff'][0]
        m0 = inputs['m0'][0]
        mdot = inputs['m_dot'][0]
        v_e = inputs['v_e'][0]
        tau = m0/mdot

        # here g_eff is calculated once, so it is an approximation.

        p1 = a0 + a1*(T-t) + a2*(T-t)**2
        p2 = p1 * (T-t)
        r_dot_dot = c1 * p1 + c2 * p2
        a_thrust = v_e/(tau - t)
        # Note: g_eff is based on time (state actually) but we will assume
        # it is static for each iteration.
        alpha = math.asin((r_dot_dot - g_eff)/(a_thrust))

        outputs['alpha'] = alpha
        outputs['a_thrust'] = a_thrust
        outputs['r_dot_dot'] = r_dot_dot
        outputs['p1'] = p1
        outputs['p2'] = p2

class FixedThrustGuidance(om.Group):
    # What is the difference between a group and an explicit component? 
    # just the name?
    def setup(self):
        self.add_subsystem('radial_control', OuterLoopRadialControl(),
                            promotes=['*'])
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])
        # self.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])
        # self.nonlinear_solver = om.NonlinearBlockGS()



class VThetaSolver(om.ExplicitComponent):
    def setup(self):
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['a0', 'a1', 'a2', 'c1', 'c2', 'sample_t', 'T',
                       'v_e', 'm0', 'm_dot', 'mu', 'r_dot_T', 'r_T']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['v_theta_T', 'v_theta_loss_T']
        for name in output_names:
            self.add_output(name, val=0.0)

    def compute(self, inputs, outputs):
        # model = om.Group()
        # model.add_subsystem('pitch_query', PitchQuery(),
        #                                  promotes=['*'])
        # pitch_query = om.Problem(model)
        # prob.setup()
        #
        # self.pitch_query_input(pitch_query, inputs)
        #
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
        r_dot_T = inputs['r_dot_T'][0]
        r_T = inputs['r_T'][0]
        #g_eff = inputs['g_eff'][0]
        tau = m0 / mdot

        r0 = np.linalg.norm(pos, axis=0)
        r_hat_0 = pos/r0
        r_dot_0 = np.dot(vel, r_hat_0)

        rot_mat = np.array([[0, -1], [1, 0]])
        theta_hat_0 = rot_mat@r_hat_0
        v_theta_0 = np.dot(vel, theta_hat_0)



        def get_time_dependent_vars(t, v_theta):
            # Note, this is not the f_matrix based on T_got used in
            # OuterLoopRadial
            t_rel = T - t

            # These values somehow messed up
            f11 = a0*t_rel + a1*t_rel**2/2 + a2*t_rel**3/3
            f21 = a0*t_rel**2/2 + a1*t_rel**3/3 + a2*t_rel**4/4
            f22 = a0*t_rel**3/3 + a1*t_rel**4/4 + a2*t_rel**5/5
            f12 = f21
            #r_dot = f11*c1 + f12*c2 + r_dot_0
            #weird thing but IT WORKS
            r_dot = r_dot_T - f11*c1 - f12*c2

            #r = f21*c1 + f22*c2 + (r0 + r_dot_0*t_rel)
            #BROKEN
            r = r_T - (f21*c1 + f22*c2 + r_dot*t_rel)

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
        
        t_span = [t0, T]
        res1 = solve_ivp(v_theta_dot, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
        res2 = solve_ivp(v_theta_dot_loss, t_span, [v_theta_0], atol=1e-9, rtol=1e-9)
        outputs['v_theta_T'] = res1.y[0, -1]

        #outputs['v_theta_loss_T'] = res2.y[0, -1]
        v_theta_T_calc = res1.y[0, -1]
        T_go = T - t0
        expected_v_theta_loss_T_calc = -v_e*math.log(1 - T_go/(tau-t0)) - (-(v_theta_T_calc - v_theta_0))
        outputs['v_theta_loss_T'] = -expected_v_theta_loss_T_calc


class TimeToGo(om.ExplicitComponent):
    def setup(self):
        self.add_input('sample_x', val=np.zeros((2)))
        self.add_input('sample_v', val=np.zeros((2)))
        input_names = ['v_theta_loss_T', 'v_theta_T', 
                       'target_v_theta_T', 'sample_t',
                       'v_e', 'm0', 'm_dot']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['T']
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
        r_hat = pos/np.linalg.norm(pos, axis=0)
        rot_mat = np.array([[0, -1], [1, 0]])
        theta_hat = rot_mat@r_hat
        v_theta_0 = np.dot(theta_hat, vel)

        tau = m0/m_dot
        tau0 = tau - t0
        v_theta_loss_Tn_1 = target_v_theta_T - v_theta_Tn + v_theta_loss_Tn
        # Normally not negative but I have done unfortunate things with the
        # sign of v_theta
        #CHANGED V_THETA_LOSS TO TN
        T_go_n_1 = tau0 * (1 - math.exp(-(-(target_v_theta_T - v_theta_0 + v_theta_loss_Tn)/v_e)))
        #T_go_n_1 = tau0 * (1 - math.exp(-(-(target_v_theta_T - v_theta_0 - v_theta_loss_Tn)/v_e)))
        T_n_1 = T_go_n_1 + t0
        
        outputs['T'] = T_n_1

class FixedThrustGuidanceFull(om.Group):
    def setup(self):
        self.add_subsystem('radial_control', OuterLoopRadialControl(),
                            promotes=['*'])
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])
        self.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])
        self.add_subsystem("time_to_go", TimeToGo(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        #self.nonlinear_solver.options['maxiter'] = 100
        #self.nonlinear_solver.options['atol'] = 1e-3


if __name__ == "__main__":

    r0 = 1737.4e3
    mu = 4.90e12
    x0 = np.array([r0, 0])
    v0 = np.array([0, 0])
    v_e = 3900
    m_dot = 0.42
    m0 = 500
    T_go_guess = 438

    model = om.Group()
    # model.add_subsystem('radial_comp', radialControl())
    model.add_subsystem('outer_loop', OuterLoopRadialControl(), promotes=['*'])
    model.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])
    model.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])
    #model.set_input_defaults('mu', mu)

    prob = om.Problem(model)
    prob.setup()
    # Initial conditions
    prob['x'] = x0
    prob['v'] = v0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 0
    prob['r_T'] = r0 + 18.52e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # Query Input
    prob['t'] = 10

    #recorder = om.SqliteRecorder('cases.sql')
    #prob.add_recorder(recorder)
    prob.run_model()
    print(prob['v_theta_T'])


