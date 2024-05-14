from abc import ABC, abstractmethod
import openmdao.api as om
import gcherry.config as cfg
from gcherry.cherry_guidance_refactor import (
    OuterLoopComponent, PitchQuery,
    RadialYawGuidance, PitchHeadingQuery
)

class GuidanceInterfaceBase(ABC):
    @abstractmethod
    def get_command(self, t, state): pass

class Test3DGuidance(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', OuterLoopComponent(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])

class GCherryGuidanceInterface(GuidanceInterfaceBase):
    _openmdao_problem: om.Problem

    def __init__(self, config: cfg.Config):
        model = Test3DGuidance()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

    def get_command(self, t, state, outer_loop=True):
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
        thrust_pitch = self._openmdao_problem['cmd_pitch'][0]
        thrust_heading = self._openmdao_problem['cmd_heading'][0]

        return thrust_magnitude, thrust_pitch, thrust_heading

    def _parse_input(self, config):
        v_e, m_dot = _convert_engine_data(config.spacecraft.specific_impulse,
                                          config.spacecraft.thrust)
        # TODO: 
        mdao_vals = [('target_r_T', config.mission.periapsis),
         ('target_r_dot_T', 0),
         ('target_lan', config.mission.longitude_of_ascending_node),
         ('target_inc', config.mission.inclination),
         ('mu', config.body.gravitational_parameter),
         ('v_e', v_e),
         ('m_dot', m_dot),
         ('m0', config.spacecraft.wet_mass),
         ('T', 438)]
        for key, value in mdao_vals:
            self._openmdao_problem.set_val(key, value)


def _convert_engine_data(specific_impulse, thrust):
    g0 = 9.80665
    exhaust_velocity = specific_impulse * g0
    mass_flow = thrust/exhaust_velocity
    return exhaust_velocity, mass_flow