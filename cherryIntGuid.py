import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from cherryIntMDAO import FixedThrustGuidance
from cherryInt import rocket_ode
from scipy.integrate import solve_ivp

def guidance_func_base(t, state, prob):
    """ Given t and state, will compute desired thrust and thrust angle.
        Inputs:
            t       double  [s] Simulation time
            state   list    Equivalent 
            prob    om.Problem  Contains FixedThrustGuidance model.
        Outputs:
            thrust_mag      double  In interval [0,1], indicates
                            percentage of max_thrust.
            thrust_angle    double  [rad.]  In interval [-pi,
                            pi]. 0 degrees indicates eastward, incre
                            asing angle counter clockwise.
                            
        Update outer guidance every second, inner guidance can be sample
        infinitely.
        Take in t, state, put into problem inputs. t goes both into
        sample_t and just t. call run_model(). Extract T. If t > T, then
        send thrust-off command and hold pitch.
        run_model() should model outer and inner loop.
    """
    altitude_hold_span = 10 #s
    T = prob['T']
    if t > T:
        thrust_mag = 0
        thrust_angle = prob['alpha']
    elif t >= T - altitude_hold_span:
        thrust_mag = 1
        thrust_angle = prob['alpha']
        # Return model values without recalculating
    else:
        # perform calculation
        x = state[0:2]
        v = state[2:4]
        m = state[4] #???
        prob['x'] = x
        prob['v'] = v
        prob['t'] = t
        # There is no estimator for m since it is perfectly known already.
        prob.run_model()
        prob.record("{:.3f}".format(t))
        thrust_mag = 1
        thrust_angle = prob['alpha']
    
    return thrust_mag, thrust_angle

if __name__ == "__main__":
    
    r0 = 1737.4e3
    mu = 4.90e12
    x0 = np.array([r0, 0])
    v0 = np.array([0, 0])
    v_e = 3900
    m_dot = 0.42
    m0 = 500
    T_go_guess = 438

    model = FixedThrustGuidance()

    prob = om.Problem(model)
    prob.setup()
    # Initial conditions
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 0
    prob['r_T'] = r0 + 18.52e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # specific gravity only for calculating Isp

    recorder = om.SqliteRecorder('rocket_ode.sql')
    prob.add_recorder(recorder)

    g0 = 9.80665
    Isp = v_e/g0
    F_thrust_max = m_dot * v_e
    initial_time = 0
    initial_state = np.concatenate((x0, v0, [m0]))
    guidance_func = lambda t, state: guidance_func_base(t, state, prob)

    deriv = rocket_ode(initial_time, initial_state, mu, Isp, F_thrust_max, 
               guidance_func)
    print(deriv)

    t_span = [0, T_go_guess]
    res = solve_ivp(rocket_ode, t_span, initial_state, args=(mu, Isp, 
                        F_thrust_max, guidance_func), t_eval=np.arange(t_span[0], t_span[1], 0.5))
    print(res)
    print("Length: {}".format(len(res.t)))
    x = res.y[0, :]
    y = res.y[1, :]
    t = res.t
    plt.plot(x, y, '-o')
    plt.show()

