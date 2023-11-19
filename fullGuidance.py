import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from cherryIntMDAO import FixedThrustGuidanceFull, PitchQuery, VThetaSolver, TimeToGo, OuterLoopRadialControl
from integrationSim import run_simulation, guidance_func_base, init_log
from cherryInt import rocket_ode

class FixedThrustGuidanceBlocks(om.Group):
    def setup(self):
        self.add_subsystem("outer_loop_full", OuterLoopFull(), promotes=['*'])
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])
        

class OuterLoopFull(om.ExplicitComponent):
    def setup(self):
        model = self.setup_model()
        
        self.prob = om.Problem(model)
        # T OVERRIDE HERE
        self.prob.model.set_input_defaults('T', 438)
        self.prob.setup()
        # recorder = om.SqliteRecorder('cases.sql')
        # self.prob.add_recorder(recorder)

        self.add_input('x', val=np.zeros((2)))
        self.add_input('v', val=np.zeros((2)))
        # SAMPLE T SHOULD BE negative interval since t0 = 0.
        input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'mu', 'v_e', 'm_dot', 'm0',
                        'target_v_theta_T']
        for name in input_names:
            self.add_input(name, val=0.0)

        self.add_output('sample_x', val=np.zeros((2)))
        self.add_output('sample_v', val=np.zeros((2)))
        # for the compute loop
        self.add_output('T', val= 438)
        output_names = ['last_sample_t', 'a0',
                        'a1', 'a2', 'c1', 'c2', 'tau', 'g_eff',
                        'v_theta_T']
        for name in output_names:
            self.add_output(name, val=0.0)
        # HARDCODE
        self.prob['T'] = 438
        
        self.last_sample_t = -1
        self.add_input('delta_end_calc', val=-10)
    
    def setup_model(self):
        model = om.Group()
        model.add_subsystem('radial_control', OuterLoopRadialControl(),
                            promotes=['*'])
        model.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])
        model.add_subsystem("time_to_go", TimeToGo(), promotes=['*'])
        model.nonlinear_solver = om.NonlinearBlockGS()
        model.nonlinear_solver.options['maxiter'] = 100
        model.nonlinear_solver.options['atol'] = 1e-3
        return model

    def compute(self, inputs, outputs):
        delta_end_calc = inputs['delta_end_calc']
        T = outputs['T']
        sample_t = inputs['sample_t']

        if sample_t < (T + delta_end_calc):
            # This should be its own function
            self.prob['x'] = inputs['x']
            self.prob['v'] = inputs['v']
            input_names = ['sample_t', 'r_dot_T',
                        'r_T', 'mu', 'v_e', 'm_dot', 'm0',
                        'target_v_theta_T']
            for name in input_names:
                self.prob[name] = inputs[name][0]


            self.prob.run_model()

            output_names = ['T', 'last_sample_t', 'a0',
                        'a1', 'a2', 'c1', 'c2', 'tau', 'g_eff',
                        'v_theta_T']
            for name in output_names:
                outputs[name] = self.prob[name]

def full_ascent(model = FixedThrustGuidanceBlocks(), log_file = "log_apollo_Tgo.pkl"):
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
     # DEBUG CHANGED FOR V_THETA_TESTS
    #T_go_guess = 448

    #model = PerfectFixedThrust()

    prob = om.Problem(model)
    prob.setup()

    # Initial conditions (Also IC for run_simulation)
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    #prob['T'] = T_go_guess
    #prob.model.set_input_defaults('T', T_go_guess)
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 9.544
    prob['r_T'] = r0 + 18.24e3 # m
    prob['target_v_theta_T'] = -1685 # m/s
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
    outer_loop_guidance_interval = 2
    eval_points = np.arange(0, 450 + 
                            outer_loop_guidance_interval/2, 
                            outer_loop_guidance_interval)
    #eval_points = [0, T_go_guess]
    #eval_points = np.linspace(0, T_go_guess, 3)
    
    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)
    
    print(ode_func(0, np.concatenate((x0, v0, [m0]))))

    run_simulation(ode_func, eval_points, prob, log, log_file)

    return prob, log
            
if __name__ == "__main__":
    full_ascent(model=FixedThrustGuidanceBlocks(), log_file = "log_apollo_Tgo.pkl")