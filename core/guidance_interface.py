import math
import copy
import openmdao.api as om
import numpy as np

# import sys, os
# sys.path.append(os.path.abspath('core'))
from cherry_guidance import FixedThrustGuidanceFull
from log_writing import init_log, log_problem

from cherry_guidance_refactor import (RadialControl, 
    PitchQuery, 
    VThetaSolver)

def _orbit_to_velocity(periapsis, apoapsis, true_anomaly, 
                        gravitational_parameter):
    mu = gravitational_parameter
    r_p = periapsis
    r_a = apoapsis
    theta = true_anomaly

    cos = math.cos
    sin = math.sin

    a = 1/2 * (r_p + r_a)
    e = 1 - r_p/a
    r = a * (1 - e**2)/(1 + e*cos(theta))

    h = (r_p * mu * (1+e))**0.5
    v_r = mu/h * e * sin(theta)
    v_theta = h/r

    return r, v_r, v_theta

def _convert_engine_data(specific_impulse, thrust):
    g0 = 9.80665
    exhaust_velocity = specific_impulse * g0
    mass_flow = thrust/exhaust_velocity
    return exhaust_velocity, mass_flow

class TestGuidance:
    def __init__(self, input_dict, terminal_time_guess = 438):
        model = FixedThrustGuidanceFull()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(input_dict)

        # Workaround for needed nonzero terminal time guess
        self._set_openmdao_problem_variable('T', terminal_time_guess)

        self.initialize_dict = {}
        self._init_log()

    def get_command(self, state, t, outer_loop=False, logging=True):
        x = state[0:2]
        v = state[2:4]
        m = state[4]
        
        if outer_loop:
            self._set_openmdao_problem_variable('sample_t', t)

        self._set_openmdao_problem_variable('x', x)
        self._set_openmdao_problem_variable('v', v)
        # is this correct?
        self._set_openmdao_problem_variable('t', t)

        self._run_openmdao_model()

        if logging:
            self._log_problem()

        thrust_magnitude = 1 # Constant thrust
        thrust_angle = self._get_openmdao_problem_variable('alpha')

        return [thrust_magnitude, thrust_angle]
    
    def _parse_input(self, input_dict):
        periapsis = input_dict['mission']['periapsis']
        apoapsis = input_dict['mission']['apoapsis']
        true_anomaly = np.deg2rad(input_dict['mission']['true_anomaly'])
        gravitational_parameter = input_dict['mission'][
            'gravitational_parameter']
        
        target_r, target_r_dot, target_v_theta = _orbit_to_velocity(
            periapsis, apoapsis, true_anomaly, gravitational_parameter)
        
        self._set_openmdao_problem_variable('r_T', target_r)
        self._set_openmdao_problem_variable('r_dot_T', target_r_dot)
        # negating v_theta_T for nefarious reasons
        self._set_openmdao_problem_variable('target_v_theta_T',
                                             -target_v_theta)


        specific_impulse = input_dict['spacecraft']['specific_impulse']
        thrust = input_dict['spacecraft']['thrust']

        exhaust_velocity, mass_flow = _convert_engine_data(
            specific_impulse, thrust)
        
        self._set_openmdao_problem_variable('mu', gravitational_parameter)
        self._set_openmdao_problem_variable('v_e', exhaust_velocity)
        self._set_openmdao_problem_variable('m_dot', mass_flow)
        # made m0 into list due to single element issue in run_simulation
        # for initial_state assignemnt
        self._set_openmdao_problem_variable('m0', [input_dict['spacecraft']['mass_total']])
    
    def _set_openmdao_problem_variable(self, variable, value):
        # Set initialized flag
        self._openmdao_problem[variable] = value

    def _get_openmdao_problem_variable(self, variable):
        return self._openmdao_problem[variable]
    
    def _run_openmdao_model(self):
        # Check that everything is initialized.
        self._openmdao_problem.run_model()

    def _init_log(self):
        self._log = init_log(self._openmdao_problem)

    def _log_problem(self):
        log_problem(self._openmdao_problem, self._log)

    def _get_log(self):
        return copy.deepcopy(self._log)
        ...
    def _log_openmdao_problem(self):
        ...
    def _check_initialize(self):
        ...

class TestRadialControlGuidance(om.Group):
    def setup(self):
        self.add_subsystem('radial_control', RadialControl(),
                            promotes=['*'])
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])
        self.add_subsystem('v_theta', VThetaSolver(), promotes=['*'])

class TestGuidance2:
    def __init__(self, input_dict, terminal_time_guess = 438):
        model = TestRadialControlGuidance()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(input_dict)

        # Workaround for needed nonzero terminal time guess
        self._set_openmdao_problem_variable('T', terminal_time_guess)

        self.initialize_dict = {}
        self._init_log()

    def get_command(self, state, t, outer_loop=False, logging=True):
        x = state[0:2]
        v = state[2:4]
        m = state[4]
        
        if outer_loop:
            self._set_openmdao_problem_variable('sample_t', t)

        self._set_openmdao_problem_variable('x', x)
        self._set_openmdao_problem_variable('v', v)
        # is this correct?
        self._set_openmdao_problem_variable('t', t)

        self._run_openmdao_model()

        if logging:
            self._log_problem()

        thrust_magnitude = 1 # Constant thrust
        thrust_angle = self._get_openmdao_problem_variable('alpha')

        return [thrust_magnitude, thrust_angle]
    
    def _parse_input(self, input_dict):
        periapsis = input_dict['mission']['periapsis']
        apoapsis = input_dict['mission']['apoapsis']
        true_anomaly = np.deg2rad(input_dict['mission']['true_anomaly'])
        gravitational_parameter = input_dict['mission'][
            'gravitational_parameter']
        
        target_r, target_r_dot, target_v_theta = _orbit_to_velocity(
            periapsis, apoapsis, true_anomaly, gravitational_parameter)
        
        self._set_openmdao_problem_variable('r_T', target_r)
        self._set_openmdao_problem_variable('r_dot_T', target_r_dot)
        # negating v_theta_T for nefarious reasons
        self._set_openmdao_problem_variable('target_v_theta_T',
                                             -target_v_theta)


        specific_impulse = input_dict['spacecraft']['specific_impulse']
        thrust = input_dict['spacecraft']['thrust']

        exhaust_velocity, mass_flow = _convert_engine_data(
            specific_impulse, thrust)
        
        self._set_openmdao_problem_variable('mu', gravitational_parameter)
        self._set_openmdao_problem_variable('v_e', exhaust_velocity)
        self._set_openmdao_problem_variable('m_dot', mass_flow)
        # made m0 into list due to single element issue in run_simulation
        # for initial_state assignemnt
        self._set_openmdao_problem_variable('m0', [input_dict['spacecraft']['mass_total']])
    
    def _set_openmdao_problem_variable(self, variable, value):
        # Set initialized flag
        self._openmdao_problem[variable] = value

    def _get_openmdao_problem_variable(self, variable):
        return self._openmdao_problem[variable]
    
    def _run_openmdao_model(self):
        # Check that everything is initialized.
        self._openmdao_problem.run_model()

    def _init_log(self):
        self._log = init_log(self._openmdao_problem)

    def _log_problem(self):
        log_problem(self._openmdao_problem, self._log)

    def _get_log(self):
        return copy.deepcopy(self._log)
        ...
    def _log_openmdao_problem(self):
        ...
    def _check_initialize(self):
        ...