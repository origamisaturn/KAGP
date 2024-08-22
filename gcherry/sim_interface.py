import numpy as np
from gcherry.rk4 import rk4
import gcherry.config as cfg
import krpc

from abc import ABC, abstractmethod
from gcherry.guidance_interface import GuidanceBase
from gcherry.log import SimulationLog
from gcherry.log_utils import almost_equal
from gcherry.transform import body2global_rot
from gcherry.rk4 import rk4_step


# TODO: add docs for this
class SimulatorBase(ABC):
    @abstractmethod
    def __init__(self, config: cfg.Config, 
                       guidance_obj: GuidanceBase): pass
    @abstractmethod
    def run(self): pass

def generateSimObj(config: cfg.Config, guidance_obj: GuidanceBase) -> SimulatorBase:
    """ Create SimulatorBase object selected by config. 
    
    Args:
        config: Config object. Contains desired simulator name.
        guidance_obj: GuidanceBase object, to be passed to simulator
            object.
        
    Returns:
        Subclass of SimulatorBase.
        
    """
    if config.integrator:
        sim_obj = IntegratorSim(config, guidance_obj)
    elif config.krpc_client:
        sim_obj = KRPCClient(config, guidance_obj)
    else:
        raise(RuntimeError("No simulation defined in config."))
    return sim_obj


class SingleStageSimulatorBase(SimulatorBase):
    """ Abstract class for implementing methods common in simulation
    objects using single stage guidance methods. """
    guidance_obj: GuidanceBase
    log: SimulationLog
    _last_outer_loop_time: float
    _outer_loop_interval: float
    _outer_loop_cutoff: float

    def _is_outer_loop_cutoff(self, t):
        """ True if less than outer_loop_cutoff seconds from guidance 
        termination. """
        T = self.guidance_obj.estimated_final_time()
        return (T - t) < self._outer_loop_cutoff
    
    def _is_outer_loop_scheduled(self, t):
        return (t - self._last_outer_loop_time) >= self._outer_loop_interval
    
    def _mark_outer_loop_calc(self, t):
        """ Sets last time outer loop was calculated. """
        self._last_outer_loop_time = t


class IntegratorSim(SingleStageSimulatorBase):
    guidance_obj: GuidanceBase
    log: SimulationLog

    # cmd_store are updated by every callback call.
    _thrust_cmd_store: float
    _pitch_cmd_store: float
    _heading_cmd_store: float

    _max_time_step: float
    # NOTE: outer loop interval will not be exact, will be slightly 
    # shifted forward by the integration timestep
    _sim_end_time: float
    _mu: float

    _initial_position: list[float]
    _initial_velocity: list[float]
    _wet_mass: float

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
        t_res = np.array([sim_start_time])
        y_res = np.array([initial_state]).T
        while t_res[-1] < self._sim_end_time - 1e-8:
            t = t_res[-1]
            y = y_res[:, -1]
            next_t = self._get_next_timestep(t)
            h = next_t - t
            next_state = np.atleast_2d(rk4_step(ode_func, t, y, h)).T

            y_res = np.concatenate((y_res, next_state), axis=1)
            t_res = np.append(t_res, next_t)
            self._integration_callback(next_t, y_res[:, -1])

        return t_res, y_res
    
    def _get_next_timestep(self, t):
        estimated_T = self.guidance_obj.estimated_final_time()
        # Values near estimated_T to have relatively sharp cutoff of 
        # throttle by integrator.
        event_times = [estimated_T - 1e-7, estimated_T + 1e-7, self._sim_end_time]
        next_time = t + self._max_time_step
        for candidate_time in event_times:
            h = candidate_time - t
            if h > 0 and h <= next_time - t:
                next_time = candidate_time
        return next_time
        


    def _integration_callback(self, t, state):
        self.log.state.log_state(t, state)
        self.guidance_obj.set_thrust_acc_measurement(t, self._get_thrust_acc(t, state))

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

    def _guidance_func_continuous(self, t, state):
        thrust_cmd, pitch_cmd, heading_cmd = self.guidance_obj.get_command(t, state, outer_loop=False, log=False)
        return thrust_cmd, pitch_cmd, heading_cmd

    def _guidance_func_discrete(self, t, state):
        """ Uses *_store variables updated by _integration_callback. """
        return self._thrust_cmd_store, self._pitch_cmd_store, self._heading_cmd_store
    
    def _get_thrust_acc(self, t, state):
        m = state[6]
        thrust_acc = self._thrust_cmd_store * self._thrust_force_max/m
        return thrust_acc



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
        F_thrust_max    double [N] Maximum possible thrust of vehicle.
        guidance_func   function    Given t and state, will compute 
                        desired thrust and thrust angle.
            Inputs:
                t       double  [s] Time
                state   list    [m, m, m, m/s, m/s, m/s, kg]
                                Elements are 
                                [x, y, z, xdot, ydot, zdot, m]
            Outputs:
                thrust_mag      double  In interval [0,1], indicates
                                percentage of max_thrust.
                thrust_pitch    double  [rad.]  In interval [-pi,
                                pi].
                thrust_yaw      double  [rad.] In interval [0, 2*pi].
                                0 is north, pi/2 is east.
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
    thrust_vector_global = (body2global_rot(0, thrust_pitch, thrust_yaw, r) 
                            @ thrust_vector_body)
    ve = Isp * g_0
    mdot = F_thrust/ve
    a_g = -mu/np.linalg.norm(r)**3 * r
    a_t = thrust_vector_global/m
    a = a_g + a_t
    return np.concatenate((v, a, [-mdot]))


class KRPCClient(SingleStageSimulatorBase):
    guidance_obj: GuidanceBase
    log: SimulationLog
    _post_guidance_measurement: int

    def __init__(self, config: cfg.Config, guidance_obj: GuidanceBase):
        self.log = SimulationLog()
        self.guidance_obj = guidance_obj
        # TODO: These should be set by config.
        self._last_outer_loop_time = 0
        self._parse_input(config)
        self._connect()
        self._init_streams()

    def run(self):
        init_time = self._conn.space_center.ut
        # guidance must start at time 0 for accurate calculation of 
        # thrust
        guidance_time = 0
        state = self._get_state()
        # Initialize outer loop solution
        self.guidance_obj.get_command(
                        0, state, outer_loop=True, log=True)
        self._mark_outer_loop_calc(guidance_time)
        estimated_T = self.guidance_obj.estimated_final_time()

        # Default initial pitch and heading
        self._vessel.auto_pilot.attenuation_angle = (0.01, 0.01, 0.01)
        self._vessel.auto_pilot.target_pitch_and_heading(90, 90)
        self._vessel.auto_pilot.target_roll = 0
        self._vessel.auto_pilot.engage()
        self._vessel.control.throttle = 1
 
        while guidance_time < estimated_T + self._post_guidance_measurement:
            state = self._get_state()
            self._log_state(state, guidance_time)
            self.guidance_obj.set_thrust_acc_measurement(guidance_time, self._get_thrust_acc())
            if (not self._is_outer_loop_cutoff(guidance_time) and 
                self._is_outer_loop_scheduled(guidance_time)):
                thrust_cmd, pitch_cmd, heading_cmd = (
                    self.guidance_obj.get_command(
                        guidance_time, state, outer_loop=True, log=True, 
                        mecoshift=self._main_engine_cutoff_shift))
                self._mark_outer_loop_calc(guidance_time)
                print("Outer Loop Calculated")
            else:
                thrust_cmd, pitch_cmd, heading_cmd = (
                    self.guidance_obj.get_command(
                        guidance_time, state, outer_loop=False, log=True,
                        mecoshift=self._main_engine_cutoff_shift))

            self._vessel.control.throttle = thrust_cmd
            self._vessel.auto_pilot.target_pitch_and_heading(
                np.rad2deg(pitch_cmd), np.rad2deg(heading_cmd))

            with self._streams['time'].condition:
                self._streams['time'].wait()

            guidance_time = self._streams['time']() - init_time
            estimated_T = self.guidance_obj.estimated_final_time()

            print("{:.2f}%\t{:.2f} deg\t{:.2f} deg\t{:.1f}s\t{:.1f}s".format(
                thrust_cmd*100,
                np.rad2deg(pitch_cmd),
                np.rad2deg(heading_cmd),
                guidance_time,
                estimated_T))
            
        # while guidance_time < estimated_T + self._post_guidance_measurement:
        #     with self._streams['time'].condition:
        #         self._streams['time'].wait()
        #     guidance_time = self._streams['time']() - init_time
        #     state = self._get_state()
        #     self._log_state(state, guidance_time)
        
        self._vessel.control.throttle = 0
        self._vessel.auto_pilot.disengage()

    def _get_thrust_acc(self):
        return self._streams['thrust']()/self._streams['mass']()

    def _get_state(self):
        pos = self._streams['position']()
        vel = self._streams['velocity']()

        rhs_pos = ksp_to_rhs(pos)
        rhs_vel = ksp_to_rhs(vel)

        t = self._streams['time']()
        x = rhs_pos[0]
        y = rhs_pos[1]
        z = rhs_pos[2]
        vx = rhs_vel[0]
        vy = rhs_vel[1]
        vz = rhs_vel[2]
        m = self._streams['mass']()

        state = [x, y, z, vx, vy, vz, m]
        return state

    def _connect(self):
        """ Connect to local KRPC server. """
        self._conn = krpc.connect(name=self.client_name)
        self._vessel = self._conn.space_center.active_vessel
        
    def _init_streams(self):
        """ Initiate data streams from KRPC server. """
        conn = self._conn
        vessel = self._vessel

        self._streams = {}

        ref_frame = vessel.orbit.body.non_rotating_reference_frame
        self._streams['mass'] = conn.add_stream(getattr, vessel, 'mass')
        self._streams['position'] = conn.add_stream(vessel.position, ref_frame)
        self._streams['velocity'] = conn.add_stream(vessel.velocity, ref_frame)
        self._streams['time'] = conn.add_stream(getattr, conn.space_center, 'ut')   
        self._streams['thrust'] = conn.add_stream(getattr, vessel, 'thrust')

    def _parse_input(self, config):
        self.client_name = config.krpc_client.name
        self._outer_loop_cutoff = config.krpc_client.outer_loop_cutoff
        self._outer_loop_interval = config.krpc_client.outer_loop_interval
        self._post_guidance_measurement = config.krpc_client.post_guidance_measurement
        self._main_engine_cutoff_shift = config.krpc_client.main_engine_cutoff_shift

    def _log_state(self, state, t):
        self.log.state.log_state(t, state)


def ksp_to_rhs(coord):
    """ Converts from KSP's left-handed frame to our global right-handed
    frame.
    
    KSP global frame has center fixed at major body, has X going through
    prime-meridian and equator, Y going through north pole, and Z going
    through equator at ra = 90deg.

    Inputs:
        coord: 3-length 1-D array of coordinates in KSP frame.

    Returns:
        3-length 1-D array of coordinates in right-handed global frame.

    """
    rot_mat = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
    coord_rhs = rot_mat@coord
    return coord_rhs