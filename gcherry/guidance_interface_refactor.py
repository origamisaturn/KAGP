from abc import ABC, abstractmethod
import openmdao.api as om
import config as cfg
from cherry_guidance_refactor import OuterLoopComponent, PitchQuery

class GuidanceInterfaceBase(ABC):
    @abstractmethod
    def get_command(self, t, state): pass

class Test3DGuidance(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', OuterLoopComponent(), promotes=['*'])
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])

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
        
        self._openmdao_problem('sample_t', t)
        self._openmdao_problem('sample_x', position)
        self._openmdao_problem('sample_v', velocity)

        self._openmdao_problem('t', t)

        thrust_magnitude = 1
        thrust_pitch = self._openmdao_problem('pitch')[0]
        thrust_heading = self._openmdao_problem('heading')[0]

        return thrust_magnitude, thrust_pitch, thrust_heading



    def _parse_input(self, config):
        ...

    