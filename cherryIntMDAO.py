import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt

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

        output_names = ['last_sample_t', 'a0', 'a1', 'a2', 'c1',
                         'c2', 'tau', 'g_eff']
        for name in output_names:
            self.add_output(name, val=0.0)
        
        self.outer_loop_interval = 1 #s
        self.last_sample_t = -self.outer_loop_interval

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
    #model.set_input_defaults('mu', mu)

    prob = om.Problem(model)
    prob.setup()
    # Initial conditions
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
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
    # prob['t'] = 10

    #recorder = om.SqliteRecorder('cases.sql')
    #prob.add_recorder(recorder)
    prob.run_model()
    print(prob['alpha'])

    N = 10
    t_vals = np.linspace(0, 20, N)
    var_names = ['last_sample_t', 'alpha', 'a_thrust', 'r_dot_dot', 'p1', 'p2']
    M = len(var_names)
    alpha_vals = np.zeros((M, N))
    for i, time in enumerate(t_vals):
        prob['t'] = time
        prob.run_model()
        prob.record(str(time))
        for j, var_name in enumerate(var_names):
            alpha_vals[j, i] = prob[var_name]
    for i, var_name in enumerate(var_names):
        plt.figure()
        plt.plot(t_vals, alpha_vals[i, :])
        plt.title(var_name)
    plt.show()

#     print(prob['alpha'])
#     prob.cleanup()
#     cr = om.CaseReader("cases.sql")
#     print("test")
#     problem_cases = cr.get_cases('problem', recurse=False)
#     test_vals = []
#     for case in problem_cases:
#         test_vals.append(case['alpha'][0])
#     plt.figure()
#     plt.plot(test_vals)
#     plt.show()
#     #print(prob['radial_comp.alpha'])
