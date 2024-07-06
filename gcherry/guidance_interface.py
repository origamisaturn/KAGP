import openmdao.api as om
import gcherry.config as cfg

from abc import ABC, abstractmethod
from gcherry.log import LogAnalyzer, GuidanceLog
from gcherry.guidance_components import (
    OrbitGuidanceComponent,
    PitchHeadingQuery,
    VThetaSolverOuterLoop
)


def generateGuidanceObj(config: cfg.Config):
    """ Returns GuidanceBase subclass depending on desired
    guidance method.
    
    Args:
        config: Config object. Contains desired guidance method.
        
    Returns:
        Subclass of GuidanceBase.

    """
    if config.orbit_targeting_ascent:
        model = OrbitTargetingAscent(config)
    elif config.debug_ascent_1:
        model = DebugAscent1(config)
    else:
        raise NotImplementedError("No recognized guidance method" +
                                    "found in config.")
    return model

# TODO: Add docs for this
class GuidanceBase(ABC):
    @abstractmethod
    def get_command(self, t, state, outer_loop=True, log=True): pass

    @abstractmethod
    def estimated_final_time(self): pass

class OpenMDAOGuidanceBase(GuidanceBase):
    """ Abstract class for implementing methods common in guidance
    objects using OpenMDAO. """
    _openmdao_problem: om.Problem
    log: GuidanceLog

    def get_command(self, t, state, outer_loop=True, log=True):
        position = state[:3]
        velocity = state[3:6]
        mass = state[6]

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


class OrbitTargetingAscentGroup(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', OrbitGuidanceComponent(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class OrbitTargetingAscent(OpenMDAOGuidanceBase):
    # _openmdao_problem: om.Problem
    # log: GuidanceLog
    def __init__(self, config: cfg.Config):
        model = OrbitTargetingAscentGroup()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = GuidanceLog()
        self.log.init_problem(self._openmdao_problem)

    # See OpenMDAOGuidanceBase
    # def get_command(self, t, state, outer_loop=True, log=True)
    # def estimated_final_time(self)

    def _parse_input(self, config):
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
            self._openmdao_problem.set_val(key, value)

class DebugAscent1Group(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', VThetaSolverOuterLoop(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class DebugAscent1(OpenMDAOGuidanceBase):
    # _openmdao_problem: om.Problem
    # log: GuidanceLog
    def __init__(self, config: cfg.Config):
        model = DebugAscent1Group()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = GuidanceLog()
        self.log.init_problem(self._openmdao_problem)

    # See OpenMDAOGuidanceBase
    # def get_command(self, t, state, outer_loop=True, log=True)
    # def estimated_final_time(self)

    def _parse_input(self, config):
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
            self._openmdao_problem.set_val(key, value)

def _convert_engine_data(specific_impulse, thrust):
    g0 = 9.80665
    exhaust_velocity = specific_impulse * g0
    mass_flow = thrust/exhaust_velocity
    return exhaust_velocity, mass_flow