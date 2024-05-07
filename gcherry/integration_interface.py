import numpy as np
from gcherry.rk4 import rk4
import config as cfg


# class SpacecraftState():
#     _position: 
#     _velocity:
#     _mass:

#     def __init__():


class IntegrationInterface():
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
    # make an abstract class to represent this
    guidance_interface: GuidanceInterfaceBase
    # log_interface: LogInterface
    _last_outer_loop_time: float

    # have to make a Config class
    _config: config


    def __init__(self, config: cfg.Config, 
                       guidance_interface: GuidanceInterfaceBase):
        self._thrust_cmd_store = 0.0
        self._pitch_cmd_store = np.deg2rad(90)
        self._heading_cmd_store = np.deg2rad(0)
        self._max_time_step = 1.0
        self.guidance_interface = guidance_interface
        self._parse_input()
        ...

    def _parse_input(self):
        self._outer_loop_interval = self._config.mission.outer_loop_interval
        self._outer_loop_cutoff = self._config.mission.outer_loop_cutoff
        self._sim_end_time = self._config.integrator.simulation_end_time
        self._mu = self._config.mission.gravitational_parameter

        self._initial_position = self._config.integrator.initial_position
        self._initial_velocity = self._config.integrator.initial_velocity
        self._isp = self._config.spacecraft.specific_impulse
        self._thrust_force_max = self._config.spacecraft.thrust
        self._wet_mass = self._config.spacecraft.wet_mass



    def run(self):
        sim_start_time = 0
        tspan = [sim_start_time, self._sim_end_time]

        initial_state = self._initial_position + self._initial_velocity + [self._wet_mass]

        ode_func = lambda t, state: rocket_ode(t, state, 
                                               self._mu, 
                                               self._isp, 
                                               self._thrust_force_max, 
                                               self._guidance_continuous)
        t_res, y_res = rk4(ode_func, tspan, initial_state, self._max_time_step, callback=self._integration_callback)


    """ How can I keep the guidance function constant between every timestep, 
    but continuous as an option?
    The callback function is my method for determining the completion of 
    time steps and for setting outer loop calculation. 
    Have two guidance funcs, one that reads from IntegrationInterface variables
    that are updated every callback, and another that reads directly from
    the GuidanceInterface.get_command(). """
    def _integration_callback(self, t, state):
        if (not self._is_outer_loop_cutoff(t) and 
            self._is_outer_loop_scheduled(t)):
            thrust_cmd, pitch_cmd, heading_cmd = (
                self.guidance_interface.get_command(t, state, outer_loop=True))
            self._mark_outer_loop_calc(t)
        else:
            thrust_cmd, pitch_cmd, heading_cmd = (
                self.guidance_interface.get_command(t, state, outer_loop=False))

        self.thrust_command_store = thrust_cmd
        self.pitch_command_store = pitch_cmd
        self.heading_command_store = heading_cmd

    # Is it less than outer_loop_cutoff seconds from guidance termination?
    def _is_outer_loop_cutoff(self, t):
        T = self.guidance_interface.estimated_final_time()
        return (T - t) < self.outer_loop_cutoff
    
    # is it time to run outer loop
    def _is_outer_loop_scheduled(self, t):
        return (t - self.last_outer_loop_time) >= self.outer_loop_interval
    
    # mark outer loop for _is_outer_loop* functions
    def _mark_outer_loop_calc(self, t):
        self.last_outer_loop_time = t

    def _guidance_func_continuous(self, t, state):
        thrust_cmd, pitch_cmd, heading_cmd = self.guidance_interface.get_command(t, state, outer_loop=False)
        return thrust_cmd, pitch_cmd, heading_cmd

    # this relies on the *_store variables being updated by the 
    # _integration_callback function.
    def _guidance_func_discrete(self, t, state):
        return self.thrust_cmd_store, self.pitch_cmd_store, self.heading_cmd_store


def Rx(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[1, 0, 0], 
                     [0, c1, -s1],
                     [0, s1, c1]])

def Ry(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[c1, 0, s1], 
                     [0, 1, 0],
                     [-s1, 0, c1]])

def Rz(angle: float):
    c1 = np.cos(angle)
    s1 = np.sin(angle)
    return np.array([[c1, -s1, 0], 
                     [s1, c1, 0],
                     [0, 0, 1]])



""" Body: Frame fixed to vehicle, rotates with the vehicle. X is 
        forward, Y to the right, Z down.
    Topocentric: Frame origin at vehicle, axes direction dependent 
        on global location. X is North, Y is East, Z is toward the global origin.
    Global: Frame origin at center of celestial body. Can be inertial 
        or rotating with celestial body. X is lon 0 lat 0, Y is longitude 90 lat 0,
        Z is lat 90.
    """
def get_lat_lon(pos_global):
    """ Gets latitude and longitude of given position. """
    x, y, z = tuple(pos_global)
    lon = np.arctan2(y, x)
    lat = np.arctan2(z, np.linalg.norm([x, y]))
    return lat, lon

def body2topo_rot(roll, pitch, yaw):
    """ Rotation from body to topocentric frame. """
    return Rz(yaw)@Ry(pitch)@Rx(roll)

def topo2body_rot(roll, pitch, yaw):
    """ Rotation from topocentric to body frame. """
    return body2topo_rot(roll, pitch, yaw).T

def topo2global_rot(lat, lon):
    """ Rotation from topocentric to global frame. """
    axes_switch_rot = np.array([[0, 0, -1],
                                [0, 1, 0],
                                [1, 0, 0]])
    # latitude negated as positive y rot is downwards
    return Rz(lon)@Ry(-lat)@axes_switch_rot

def global2topo_rot(lat, lon):
    """ Rotation from global to topocentric frame. """
    return topo2global_rot(lat, lon).T

def body2global_rot(roll, pitch, yaw, pos_global):
    lat, lon = get_lat_lon(pos_global)
    return topo2global_rot(lat, lon) @ body2topo_rot(roll, pitch, yaw)

def global2body_rot(roll, pitch, yaw, pos_global):
    return body2global_rot(roll, pitch, yaw, pos_global).T


# maybe avoid adding input verification for performance.
def rocket_ode(t, state, mu, Isp, F_thrust_max, guidance_func):
    """ Models single-stage rocket in 3 dimensions.
    
    Derivative of state with respect for time for use in the solve_ivp
    function. Models a rocket under the influence of
    gravity in 3 dimensions, with instaneous turning.
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
                    thrust_yaw      double  [rad.] TBD.
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