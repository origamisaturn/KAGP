from scipy.integrate import solve_ivp
import numpy as np
import math

def generate_rocket_ode_2(t, state, mu, Isp, F_thrust_max, guidance_interface):
    """ used for integrating into rk4. state is:
        [x, y, xdot, ydot, m]
        angle of elevation is from guidance function inner loop, no
        outer loop calculation."""
    

# 
def rocket_ode(t, state, mu, Isp, F_thrust_max, guidance_func):
    """ Derivative of state with respect for time for use in the solve_ivp
    function. Models a vehicle with rocket thrust under the influence of
    gravity in 2 dimensions.
        Inputs:
            t       double  [s] Time
            state   list    [m, m, m/s, m/s, kg]
                            Elements are [x, y, xdot, ydot, m]
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
                    thrust_angle    double  [rad.]  In interval [-pi,
                                    pi]. 0 degrees indicates eastward, incre                                    asing angle counter clockwise.
        Outputs:
            statedot    list    [m/s, m/s, m/s^2, m/s^2, kg/s]
                                Elements are [xdot, ydot, xdotdot, ydotdot, 
                                mdot]
    """
    g_0 = 9.80665   # m/s^2
    x = state[0]
    y = state[1]
    xdot = state[2]
    ydot = state[3]
    m = state[4]

    thrust_mag, thrust_angle = guidance_func(t, state)

    position_angle = math.atan2(y, x)
    # converts local thrust_angle above horizon to angle in global frame.
    global_angle = position_angle - 90*math.pi/180 + thrust_angle

    r = np.array([x, y])
    v = np.array([xdot, ydot])

    F_thrust = F_thrust_max * thrust_mag
    ve = Isp * g_0
    mdot = F_thrust/ve

    a_g = -mu/np.linalg.norm(r)**3 * r
    a_t = F_thrust/m * np.array([math.cos(global_angle), math.sin(global_angle)])
    a = a_g + a_t
    return np.concatenate((v, a, [-mdot]))


def gravity_ode(t, state, mu):
    x = state[0]
    y = state[1]
    xdot = state[2]
    ydot = state[3]

    r = np.array([x, y])
    v = np.array([xdot, ydot])

    a = -mu/np.linalg.norm(r)**3 * r
    return np.concatenate((v, a))


# lunar_radius = 1740e3
# earth_radius = 6378.1370e3
# theta = 0
# lunar_state = [lunar_radius*math.cos(theta), lunar_radius*math.sin(theta), 0, 0]
# earth_state = [earth_radius*math.cos(theta), earth_radius*math.sin(theta), 0, 0]

# t = 0
# mu_lunar = 4.9048695e12 #Lunar gravitational parameter
# mu_earth = 3.986004418e14
# t_span = [0, 4]
#print(gravity_ode(t, earth_state, mu_earth))
#print(solve_ivp(gravity_ode, t_span, earth_state, args=(mu_earth,)))
