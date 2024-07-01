import openmdao.api as om
import gcherry.config as cfg

from abc import ABC, abstractmethod
from gcherry.log_interface import LogInterfaceRefactor, GuidanceInterfaceLog
from gcherry.cherry_guidance_refactor import (
    OrbitGuidanceComponent,
    PitchHeadingQuery,
    VThetaSolverOuterLoop
)

# TODO: Add docs for this, add keyword "run outer loop" ?
class GuidanceInterfaceBase(ABC):
    @abstractmethod
    def get_command(self, t, state): pass

    @abstractmethod
    def estimated_final_time(self): pass

class OrbitTargetingAscentGroup(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', OrbitGuidanceComponent(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class DebugAscent1Group(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', VThetaSolverOuterLoop(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class GCherryGuidanceInterface(GuidanceInterfaceBase):
    _openmdao_problem: om.Problem
    log: GuidanceInterfaceLog

    def __init__(self, config: cfg.Config, log_interface: LogInterfaceRefactor):
    # TODO: replace this with something else
        if config.orbit_targeting_ascent:
            model = OrbitTargetingAscentGroup()
        elif config.debug_ascent_1:
            model = DebugAscent1Group()
        else:
            raise NotImplementedError("No recognized guidance method" +
                                      "found in config.")
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = log_interface.guidance_interface
        self.log.init_problem(self._openmdao_problem)

    # NOTE: calling outer_loop after estimated_T is arrived at is 
    # undefined behavior
    # TODO: Consider using a factory design pattern
    def get_command(self, t, state, outer_loop=True, log=True):
        # TODO: replace this with a state or a state factory, like in Poliastro.
        # or maybe not, might be too complicated
        position = state[:3]
        velocity = state[3:6]
        mass = state[6]

        # TODO: outer loop stuff??
        
        if outer_loop == True:
            self._openmdao_problem['run_outer_loop'] = True
            self._openmdao_problem['sample_t'] = t
            self._openmdao_problem['sample_x'] = position
            self._openmdao_problem['sample_v'] = velocity
        else:
            self._openmdao_problem['run_outer_loop'] = False

        self._openmdao_problem['query_t'] = t
        self._openmdao_problem['query_x'] = position
        self._openmdao_problem['query_v'] = velocity

        self._openmdao_problem.run_model()

        thrust_magnitude = 1
        if t > self._openmdao_problem['T']:
            thrust_magnitude = 0
        thrust_pitch = self._openmdao_problem['cmd_pitch'][0]
        thrust_heading = self._openmdao_problem['cmd_heading'][0]

        if log:
            self.log.log_problem(self._openmdao_problem)

        return thrust_magnitude, thrust_pitch, thrust_heading
    
    def estimated_final_time(self):
        return self._openmdao_problem['T'][0]

    # TODO: consider making this into a separate class interface
    def _parse_input(self, config):
        if config.orbit_targeting_ascent:
            _parse_input_orbit_targeting_ascent(self._openmdao_problem, config)
        elif config.debug_ascent_1:
            _parse_input_debug_ascent_1(self._openmdao_problem, config)
        else:
            raise NotImplementedError("No recognized guidance method" +
                                      "found in config.")

def _convert_engine_data(specific_impulse, thrust):
    g0 = 9.80665
    exhaust_velocity = specific_impulse * g0
    mass_flow = thrust/exhaust_velocity
    return exhaust_velocity, mass_flow

def _parse_input_orbit_targeting_ascent(openmdao_problem, config):
    v_e, m_dot = _convert_engine_data(config.spacecraft.specific_impulse,
                                      config.spacecraft.thrust)
    mdao_vals = [
        ('target_pe', config.orbit_targeting_ascent.periapsis),
        ('target_ap', config.orbit_targeting_ascent.apoapsis),
        ('target_argp', config.orbit_targeting_ascent.argument_of_periapsis),
        ('target_lan', config.orbit_targeting_ascent.longitude_of_ascending_node),
        ('target_inc', config.orbit_targeting_ascent.inclination),
        ('mu', config.body.gravitational_parameter),
        ('v_e', v_e),
        ('m_dot', m_dot),
        ('m0', config.spacecraft.wet_mass)]
    for key, value in mdao_vals:
        openmdao_problem.set_val(key, value)

def _parse_input_debug_ascent_1(openmdao_problem, config):
    v_e, m_dot = _convert_engine_data(config.spacecraft.specific_impulse,
                                      config.spacecraft.thrust)
    mdao_vals = [
        ('T', config.debug_ascent_1.terminal_time),
        ('target_r_T', config.debug_ascent_1.radius),
        ('target_r_dot_T', config.debug_ascent_1.radial_velocity),
        ('target_lan', config.debug_ascent_1.longitude_of_ascending_node),
        ('target_inc', config.debug_ascent_1.inclination),
        ('mu', config.body.gravitational_parameter),
        ('v_e', v_e),
        ('m_dot', m_dot),
        ('m0', config.spacecraft.wet_mass)]
    for key, value in mdao_vals:
        openmdao_problem.set_val(key, value)


class TestVThetaSolver(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', VThetaSolverOuterLoop(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class VThetaSolverTestInterface(GuidanceInterfaceBase):
    _openmdao_problem: om.Problem
    log: GuidanceInterfaceLog

    def __init__(self, config: cfg.Config, log_interface: LogInterfaceRefactor):
        model = TestVThetaSolver()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = log_interface.guidance_interface
        self.log.init_problem(self._openmdao_problem)

    # NOTE: calling outer_loop after estimated_T is arrived at is 
    # undefined behavior
    def get_command(self, t, state, outer_loop=True, log=True):
        # TODO: replace this with a state or a state factory, like in Poliastro.
        # or maybe not, might be too complicated
        position = state[:3]
        velocity = state[3:6]
        mass = state[6]

        # TODO: outer loop stuff??
        
        if outer_loop == True:
            self._openmdao_problem['run_outer_loop'] = True
            self._openmdao_problem['sample_t'] = t
            self._openmdao_problem['sample_x'] = position
            self._openmdao_problem['sample_v'] = velocity
        else:
            self._openmdao_problem['run_outer_loop'] = False

        self._openmdao_problem['query_t'] = t
        self._openmdao_problem['query_x'] = position
        self._openmdao_problem['query_v'] = velocity

        self._openmdao_problem.run_model()

        thrust_magnitude = 1
        if t > self._openmdao_problem['T']:
            thrust_magnitude = 0
        thrust_pitch = self._openmdao_problem['cmd_pitch'][0]
        thrust_heading = self._openmdao_problem['cmd_heading'][0]

        if log:
            self.log.log_problem(self._openmdao_problem)

        return thrust_magnitude, thrust_pitch, thrust_heading
    
    def estimated_final_time(self):
        return self._openmdao_problem['T'][0]

    def _parse_input(self, config):
        v_e, m_dot = _convert_engine_data(config.spacecraft.specific_impulse,
                                          config.spacecraft.thrust)
        mdao_vals = [
         ('T', config.debug_guidance_1.terminal_time),
         ('target_r', config.debug_guidance_1.target_r_T),
         ('target_r_dot_T', config.debug_guidance_1.target_r_dot_T),
         ('target_lan', config.debug_guidance_1.longitude_of_ascending_node),
         ('target_inc', config.debug_guidance_1.inclination),
         ('mu', config.body.gravitational_parameter),
         ('v_e', v_e),
         ('m_dot', m_dot),
         ('m0', config.spacecraft.wet_mass)]
        for key, value in mdao_vals:
            self._openmdao_problem.set_val(key, value)
