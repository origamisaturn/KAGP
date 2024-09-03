from abc import ABC, abstractmethod
from typing_extensions import override
import openmdao.api as om

import gcherry.config as cfg
from gcherry.log import GuidanceLog
from gcherry.guidance_components import (
    OrbitGuidanceComponent,
    PitchHeadingQuery,
    VThetaSolverOuterLoop,
    EnginePropertyEstimator
)


class GuidanceBase(ABC):
    """ Base class for all guidance objects. """

    @abstractmethod
    def __init__(self, config: cfg.Config):
        """ Initializes guidance based on config. 
        
        Args:
            config: Config object. Contains settings for guidance.
        """

    @abstractmethod
    def get_command(self, t, state, outer_loop=True, log=True, mecoshift=0.0):
        """ Gets orientation and thrust command.

        Args:
            t: [s] Time, a float. Guidance assumed to start at t=0.
            state: [m, m, m, m/s, m/s, m/s, kg] A list, with elements 
                [x, y, z, xdot, ydot, zdot, m] representing position, 
                velocity, and mass.
            outer_loop: bool. If true, will run the outer loop of the
                guidance algorithm.
            log: bool. If true, will log values for current function
                call.
            mecoshift: float, short for 'main engine cutoff shift.' 

        Returns:
            (thrust_magnitude, thrust_pitch, thrust_heading) 
            [n/a, rad., rad.] 3-tuple of floats.
        """

    @abstractmethod
    def set_thrust_acc_measurement(self, t, thrust_acc):
        """ Provides sample thrust acceleration measurement.

        Measurement is processed when get_command() is called.

        Args:
            t: [s] Time, a float.
            thrust_acc: [m/s^2] float, thrust acceleration at time t.
        """

    @abstractmethod
    def estimated_final_time(self):
        """ Gets estimated time at guidance cut-off. """


def generate_guidance_obj(config: cfg.Config) -> GuidanceBase:
    """ Returns GuidanceBase object selected by config
    
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


class OpenMDAOGuidanceBase(GuidanceBase):
    """ Abstract class for implementing methods common in guidance
    objects using OpenMDAO. 
    
    Attributes:
        log: GuidanceLog object for logging guidance values.
    """
    log: GuidanceLog

    _openmdao_problem: om.Problem
    _estimated_T: float

    @abstractmethod
    def __init__(self, config: cfg.Config):
        self._estimated_T = None

    @override
    def get_command(self, t, state, outer_loop=True, log=True, mecoshift=0.0):
        # First call sets up the terminal time T

        # Uses self._estimated_T instead of self._openmdao_problem['T']
        # directly as it is undefined in the openmdao problem before the
        # first call to get_command.
        if (self._estimated_T is None or
            t <= self._estimated_T + mecoshift):

            position = state[:3]
            velocity = state[3:6]
            # mass = state[6]

            if outer_loop:
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

            self._estimated_T = self._openmdao_problem['T']
            thrust_magnitude = 1

        else:
            thrust_magnitude = 0

        thrust_pitch = self._openmdao_problem['cmd_pitch'][0]
        thrust_heading = self._openmdao_problem['cmd_heading'][0]

        if log:
            self.log.log_guidance(self._openmdao_problem, thrust_magnitude)

        return thrust_magnitude, thrust_pitch, thrust_heading

    @override
    def estimated_final_time(self):
        return self._openmdao_problem['T'][0]


class OrbitTargetingAscentGroup(om.Group):
    def setup(self):
        self.add_subsystem('engine_estimator', EnginePropertyEstimator(), promotes=['*'])
        self.add_subsystem('outer_loop', OrbitGuidanceComponent(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])


class OrbitTargetingAscent(OpenMDAOGuidanceBase):
    """ Guidance object for targeting an orbit. """
    _enable_estimator: bool

    @override
    def __init__(self, config: cfg.Config):
        model = OrbitTargetingAscentGroup()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = GuidanceLog()
        self.log.init_problem(self._openmdao_problem)
        super().__init__(config)

    @override
    def get_command(self, t, state, outer_loop=True, log=True, mecoshift=0.0):
        if outer_loop and self._enable_estimator:
            self._openmdao_problem['run_estimator'] = True
        else:
            self._openmdao_problem['run_estimator'] = False
        return super().get_command(
            t, state,
            outer_loop=outer_loop, log=log, mecoshift=mecoshift)

    @override
    def set_thrust_acc_measurement(self, t, thrust_acc):
        self._openmdao_problem['estimator_sample_t'] = t
        self._openmdao_problem['sample_thrust_acceleration'] = thrust_acc

    def _parse_input(self, config):
        self._enable_estimator = config.orbit_targeting_ascent.enable_estimator
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
            ('m0', config.spacecraft.wet_mass),
            ('estimator_ignore_time', config.orbit_targeting_ascent.estimator_ignore_time),
            ('estimator_output_time', config.orbit_targeting_ascent.estimator_output_time)]
        for key, value in mdao_vals:
            self._openmdao_problem.set_val(key, value)


class DebugAscent1Group(om.Group):
    def setup(self):
        self.add_subsystem('outer_loop', VThetaSolverOuterLoop(), promotes=['*'])
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])


class DebugAscent1(OpenMDAOGuidanceBase):
    """ Guidance object for debugging the VThetaSolver component. """

    @override
    def __init__(self, config: cfg.Config):
        model = DebugAscent1Group()
        self._openmdao_problem = om.Problem(model)
        self._openmdao_problem.setup()
        self._parse_input(config)

        self.log = GuidanceLog()
        self.log.init_problem(self._openmdao_problem)
        super().__init__(config)

    @override
    def set_thrust_acc_measurement(self, t, thrust_acc):
        pass

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
