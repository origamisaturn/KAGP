from ode import rocket_ode
import numpy as np
from integration_sim import init_log, run_simulation, guidance_func_base

class Integrator2DInterface:
    def __init__(self, input_dict, guidance_interface):
        self.guidance_interface = guidance_interface
        self._init_log()
        self._parse_input(input_dict)
        ...
    def run(self):
        run_simulation(self._ode_func,
                self._eval_points, 
                self.guidance_interface._openmdao_problem, 
                self._log, 
                self._log_path)
        ...
    def _parse_input(self, input_dict):
        # 02/20/24 ADD INPUT FOR INITIAL STATE IN SIMULATION
        init_pos = input_dict['simulator']['initial_position']
        init_vel = input_dict['simulator']['initial_velocity']
        init_m = input_dict['spacecraft']['mass_total']
        # self._state = np.concatenate((init_pos, init_vel, init_m))
        self.guidance_interface._openmdao_problem['x'] = init_pos
        self.guidance_interface._openmdao_problem['v'] = init_vel

        self._t = 0
        self._log_path = input_dict['simulator']['log_path']

        outer_loop_period = input_dict['simulator']['outer_loop_period']
        sim_end_time = input_dict['simulator']['simulation_end_time']
        self._eval_points = np.arange(0, sim_end_time + outer_loop_period/2,
                                      outer_loop_period)

        # might want to move gravitational parameter
        mu = input_dict['mission']['gravitational_parameter']
        isp = input_dict['spacecraft']['specific_impulse']
        F_thrust_max = input_dict['spacecraft']['thrust']
        guidance_func = lambda t, state: self.guidance_interface.get_command(state, t)
        self._ode_func = lambda t, state: rocket_ode(
            t, state, mu, isp, F_thrust_max, guidance_func)
        ...
    def _init_log(self):
        self._log = init_log(self.guidance_interface._openmdao_problem)
        ...
    def _log_res(self):
        ...

