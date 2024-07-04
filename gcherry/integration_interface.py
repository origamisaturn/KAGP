import numpy as np
from gcherry.rk4 import rk4
import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GuidanceBase
from gcherry.log_interface import SimulationLog
from gcherry.transform import body2global_rot


# need from_state_vector and from_components
class SpacecraftState():
    _position: list[float]
    _velocity: list[float]
    _mass: float

    def __init__(self, state_vector):
        self._position = list(state_vector[0:3])
        self._velocity = list(state_vector[3:6])
        self._mass = state_vector[6]

    def position(self):
        return self._position
    
    def velocity(self):
        return self._velocity
    
    def mass(self):
        return self._mass
    
    def state_vector(self):
        return self._position + self._velocity + [self._mass]


class IntegrationInterface():
    guidance_obj: GuidanceBase
    log: SimulationLog

    # These are updated by every callback call.
    _thrust_cmd_store: float
    _pitch_cmd_store: float
    _heading_cmd_store: float

    _max_time_step: float
    # will not be exact, will be shifted forward
    # by the integration timestep
    _outer_loop_interval: float
    _outer_loop_cutoff: float
    _sim_end_time: float
    _mu: float

    _initial_position: list[float]
    _initial_velocity: list[float]
    _wet_mass: float

    _last_outer_loop_time: float

    def __init__(self, config: cfg.Config, 
                       guidance_obj: GuidanceBase):
        self._thrust_cmd_store = 0.0
        self._pitch_cmd_store = np.deg2rad(90)
        self._heading_cmd_store = np.deg2rad(0)
        self._max_time_step = 1.0
        self._last_outer_loop_time = 0.0
        self.guidance_obj = guidance_obj
        self.log = SimulationLog()
        self._parse_input(config)

    def _parse_input(self, config: cfg.Config):
        self._isp = config.spacecraft.specific_impulse
        self._thrust_force_max = config.spacecraft.thrust
        self._wet_mass = config.spacecraft.wet_mass

        self._outer_loop_interval = config.integrator.outer_loop_interval
        self._outer_loop_cutoff = config.integrator.outer_loop_cutoff

        self._mu = config.body.gravitational_parameter

        self._initial_position = config.integrator.initial_position
        self._initial_velocity = config.integrator.initial_velocity
        self._sim_end_time = config.integrator.simulation_end_time

    def run(self):
        # guidance func start time should not be anything other than 0.
        sim_start_time = 0
        tspan = [sim_start_time, self._sim_end_time]

        initial_state = self._initial_position + self._initial_velocity + [self._wet_mass]
        self.log.state.log_state(sim_start_time, initial_state)

        # Initialize outer loop solution
        self.guidance_obj.get_command(0, initial_state, outer_loop=True, log=True)

        ode_func = lambda t, state: rocket_ode(t, state, 
                                               self._mu, 
                                               self._isp, 
                                               self._thrust_force_max, 
                                               self._guidance_func_continuous)
        # NOTE: consider using rk4_step instead in order to change one of
        # the integration steps to be the estimated final time. That way
        # there is no thrusting past the estimated final time.
        t_res, y_res = rk4(ode_func, tspan, initial_state, self._max_time_step, callback=self._integration_callback)
        print(np.linalg.norm(y_res[:3, -1]))
        return t_res, y_res


    """ How can I keep the guidance function constant between every timestep, 
    but continuous as an option?
    The callback function is my method for determining the completion of 
    time steps and for setting outer loop calculation. 
    Have two guidance funcs, one that reads from IntegrationInterface variables
    that are updated every callback, and another that reads directly from
    the GuidanceInterface.get_command(). """
    def _integration_callback(self, t, state):
        self.log.state.log_state(t, state)

        if (not self._is_outer_loop_cutoff(t) and 
            self._is_outer_loop_scheduled(t)):
            thrust_cmd, pitch_cmd, heading_cmd = (
                self.guidance_obj.get_command(t, state, outer_loop=True, log=True))
            self._mark_outer_loop_calc(t)
        else:
            thrust_cmd, pitch_cmd, heading_cmd = (
                self.guidance_obj.get_command(t, state, outer_loop=False, log=True))

        self._thrust_command_store = thrust_cmd
        self._pitch_command_store = pitch_cmd
        self._heading_command_store = heading_cmd

    # Is it less than outer_loop_cutoff seconds from guidance termination?
    def _is_outer_loop_cutoff(self, t):
        T = self.guidance_obj.estimated_final_time()
        return (T - t) < self._outer_loop_cutoff
    
    # is it time to run outer loop
    def _is_outer_loop_scheduled(self, t):
        return (t - self._last_outer_loop_time) >= self._outer_loop_interval
    
    # mark outer loop for _is_outer_loop* functions
    def _mark_outer_loop_calc(self, t):
        self._last_outer_loop_time = t

    def _guidance_func_continuous(self, t, state):
        thrust_cmd, pitch_cmd, heading_cmd = self.guidance_obj.get_command(t, state, outer_loop=False, log=False)
        return thrust_cmd, pitch_cmd, heading_cmd

    # this relies on the *_store variables being updated by the 
    # _integration_callback function.
    def _guidance_func_discrete(self, t, state):
        return self._thrust_cmd_store, self._pitch_cmd_store, self._heading_cmd_store


# maybe avoid adding input verification for performance.
def rocket_ode(t, state, mu, Isp, F_thrust_max, guidance_func):
    """ Models single-stage rocket in 3 dimensions.
    
    Derivative of state with respect for time for use in the solve_ivp
    function. Models a rocket under the influence of
    gravity in 3 dimensions, with instantaneous turning.
    
        Inputs:
            t       double  [s] Time
            state   list    [m, m, m, m/s, m/s, m/s, kg]
                            Elements are [x, y, z, xdot, ydot, zdot, m]
            mu      double  [m^3/s^2] Gravitational parameter
            Isp     double  [s] Specific impulse of rocket.
            F_thrust_max    [N] Maximum possible thrust of vehicle.
            guidance_func   function    Given t and state, will compute 
                            desired thrust and thrust angle.
                Inputs:
                    t       double  [s] Time
                    state   list    Equivalent as above
                Outputs:
                    thrust_mag      double  In interval [0,1], indicates
                                    percentage of max_thrust.
                    thrust_pitch    double  [rad.]  In interval [-pi,
                                    pi]. 0 degrees indicates eastward, incre                                    
                                    asing angle counter clockwise.
                    thrust_yaw      double  [rad.] TBD. TODO.
        Outputs:
            statedot    list    [m/s, m/s, m/s, m/s^2, m/s^2, m/s^2, kg/s]
                                Elements are [xdot, ydot, zdot, xdotdot, 
                                ydotdot, zdotdot, mdot]
    """
    g_0 = 9.80665   # m/s^2

    r = np.array((state[0], state[1], state[2]))
    v = np.array((state[3], state[4], state[5]))
    m = state[6]

    thrust_mag, thrust_pitch, thrust_yaw = guidance_func(t, state)

    F_thrust = F_thrust_max * thrust_mag
    thrust_vector_body = np.array([F_thrust, 0, 0])
    # TODO: Test the rotations.
    thrust_vector_global = (body2global_rot(0, thrust_pitch, thrust_yaw, r) 
                            @ thrust_vector_body)

    ve = Isp * g_0
    mdot = F_thrust/ve

    a_g = -mu/np.linalg.norm(r)**3 * r
    a_t = thrust_vector_global/m
    a = a_g + a_t
    return np.concatenate((v, a, [-mdot]))