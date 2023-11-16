import openmdao.api as om
import numpy as np
import math
from cherryIntMDAO import OuterLoopRadialControl, VThetaSolver
from integrationSim import run_simulation, guidance_func_base, init_log
from cherryInt import rocket_ode

def rot_mat_2d(theta):
    cos = math.cos
    sin = math.sin
    rot_mat = np.array([[cos(theta), -sin(theta)],
        [sin(theta), cos(theta)]])
    return rot_mat

class PerfectPitchQuery(om.ExplicitComponent):
    """
    Component containing the equation for pitch of the fixed-thrust control
    law. Accepts equation terms from the radialControl block. This is
    called at a much higher rate than radialControl, which is called whenver 
    we want to update Tgo (at approximately 1hz).
    """

    def setup(self):
        self.add_input('x', val=np.zeros((2)))
        self.add_input('v', val=np.zeros((2)))
        input_names = ['t', 'T', 'a0', 'a1', 'a2', 'c1', 'c2', 
                       'm0', 'm_dot', 'v_e', 'mu']
        for name in input_names:
            self.add_input(name, val=0.0)
        
        output_names = ['alpha', 'a_thrust', 'r_dot_dot', 'p1', 'p2']
        for name in output_names:
            self.add_output(name, val=0.0)

    # Did not add setup_partials
    def compute(self, inputs, outputs):
        # used purely for g_eff calculations, assumed updated every time
        # so that sample_t = t.
        x0 = inputs['x']
        v0 = inputs['v']

        t = inputs['t'][0]
        T = inputs['T'][0]
        a0 = inputs['a0'][0]
        a1 = inputs['a1'][0]
        a2 = inputs['a2'][0]
        c1 = inputs['c1'][0]
        c2 = inputs['c2'][0]
        m0 = inputs['m0'][0]
        mdot = inputs['m_dot'][0]
        v_e = inputs['v_e'][0]
        mu = inputs['mu'][0]
        tau = m0/mdot

        r0 = np.linalg.norm(x0)
        r_hat_0 = x0/r0
        deg_to_pi = math.pi/180
        theta = -90 * deg_to_pi
        theta_hat_0 = (rot_mat_2d(theta)@r_hat_0.reshape((2,1))).reshape(2)

        v_theta_0 = np.dot(v0.T, theta_hat_0)


        p1 = a0 + a1*(T-t) + a2*(T-t)**2
        p2 = p1 * (T-t)
        r_dot_dot = c1 * p1 + c2 * p2
        a_thrust = v_e/(tau - t)
        # Note: g_eff is based on time (state actually) but we will assume
        # it is static for each iteration.
        g_eff = -mu/r0**2 + v_theta_0**2/r0
        alpha = math.asin((r_dot_dot - g_eff)/(a_thrust))

        outputs['alpha'] = alpha
        outputs['a_thrust'] = a_thrust
        outputs['r_dot_dot'] = r_dot_dot
        outputs['p1'] = p1
        outputs['p2'] = p2

class PerfectFixedThrust(om.Group):
    def setup(self):
        self.add_subsystem('radial_control', OuterLoopRadialControl(),
                           promotes=['*'])
        self.add_subsystem('pitch_query', PerfectPitchQuery(), 
                           promotes=['*'])         
        self.add_subsystem('v_theta_solver', VThetaSolver(), 
                           promotes=['*'])
        
def apollo_ascent(model = PerfectFixedThrust(), log_file = "log_apollo_ascent_perfect.pkl"):
    r0 = 1737.4e3
    mu = 4.90e12
    x0 = np.array([r0, 0])
    v0 = np.array([0, 2.34]) # Add rotation of moon.
    #v0 = np.array([0, 0])
    Isp = 310
    g0 = 9.80665
    v_e = Isp*g0
    F_thrust_max = 15.87e3
    m_dot = F_thrust_max / v_e
    m0 = 5100
    T_go_guess = 438

    #model = PerfectFixedThrust()

    prob = om.Problem(model)
    prob.setup()

    # Initial conditions (Also IC for run_simulation)
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 9.544
    prob['r_T'] = r0 + 18.24e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # Why do we need this?
    prob.run_model()

    # temp value for log
    log = init_log(prob)

    # Integral time boundaries based on the rate we want to recalculate 
    # the outer loop guidance block.
    outer_loop_guidance_interval = 7
    #eval_points = np.arange(0, T_go_guess + 
    #                        outer_loop_guidance_interval/2, 
    #                        outer_loop_guidance_interval)
    #eval_points = [0, T_go_guess]
    eval_points = np.linspace(0, T_go_guess, 3)
    
    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)
    
    print(ode_func(0, np.concatenate((x0, v0, [m0]))))

    run_simulation(ode_func, eval_points, prob, log, log_file)

    return prob, log

if __name__ == "__main__":
    apollo_ascent()