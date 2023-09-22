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
        self.add_input('sample_t', val=0.0)
        self.add_input('last_sample_t', val=0.0)
        self.add_input('r_dot_T', val = 0)
        self.add_input('r_T', val = 0)
        self.add_input('T', val=0.0)
        self.add_input('mu', val=0.0)
        self.add_input('v_e', val=0.0)
        self.add_input('m_dot', val=0.0)
        self.add_input('m0', val = 0.0)

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



class PitchQuery(om.ExplicitComponent):
    """
    Component containing the equation for pitch of the fixed-thrust control
    law. Accepts equation terms from the radialControl block. This is
    called at a much higher rate than radialControl, which is called whenver 
    we want to update Tgo (at approximately 1hz).
    """

    def setup(self):
        input_names = ['t', 'T', 'a0', 'a1', 'a2', 'c1', 'c2', 'g_eff', 'm0', 'm_dot']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['alpha']
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

class FixedThrustGuidance(om.Group):
    # What is the difference between a group and an explicit component? 
    # just the name?
    def setup(self):
        self.add_subsystem('radial_control', RadialControl(), promotes=['*'])
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

    # model = om.Group()
    # model.add_subsystem('radial_comp', radialControl())
    model = FixedThrustGuidance()
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

    """
      self.add_input('x', val=np.zeros((2,1)))
        self.add_input('v', val=np.zeros((2,1)))
        self.add_input('t', val = 0)
        self.add_input('r_dot_T', val = 0)
        self.add_input('r_T', val = 0)
        self.add_input('T_go', val=0.0)
        self.add_input('mu', val=0.0)
        self.add_input('v_e', val=0.0)
        self.add_input('m_dot', val=0.0)
        self.add_input('m0', val = 0.0)
        # This needs to be replaced with its own block
        self.add_output('alpha', val = 0.0)
    """
    """
    prob.set_val('radial_comp.x', x0)
    prob.set_val('radial_comp.v', v0)
    prob.set_val('radial_comp.t', 0)
    prob.set_val('radial_comp.r_dot_T', 1)
    prob.set_val('radial_comp.r_T', r0 + 100e3)
    prob.set_val('radial_comp.T_go', 820)
    prob.set_val('radial_comp.mu', mu)
    prob.set_val('radial_comp.v_e', v_e)
    prob.set_val('radial_comp.m_dot', m_dot)
    prob.set_val('radial_comp.m0', m0)
    """
    prob.run_model()

    t_vals = np.linspace(0, T_go_guess, 100)
    alpha_vals = np.zeros(100)
    for i, time in enumerate(t_vals):
        prob['t'] = time
        prob.run_model()
        alpha_vals[i] = prob['alpha']
    plt.plot(t_vals, alpha_vals)
    plt.show()

    print(prob['alpha'])
    #print(prob['radial_comp.alpha'])
