import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from cherryIntMDAO import FixedThrustGuidance, FixedThrustGuidanceFull
from cherryInt import rocket_ode
from scipy.integrate import solve_ivp
import pickle as pkl
from integrationSim import init_log, run_simulation, guidance_func_base

def apollo_ascent():
    log_file = "log_apollo_ascent.pkl"
    r0 = 1737.4e3
    mu = 4.90e12
    x0 = np.array([r0, 0])
    v0 = np.array([0, 2.34]) # Add rotation of moon.
    Isp = 310
    g0 = 9.80665
    v_e = Isp*g0
    F_thrust_max = 15.87e3
    m_dot = F_thrust_max / v_e
    m0 = 5100
    T_go_guess = 438

    model = FixedThrustGuidance()

    prob = om.Problem(model)
    prob.setup()

    # Initial conditions (Also IC for run_simulation)
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 9.544
    prob['r_T'] = r0 + 18.24e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # Why do we need this?
    prob.run_model()

    # temp value for log
    log = init_log(prob)

    # Integral time boundaries based on the rate we want to recalculate 
    # the outer loop guidance block.
    outer_loop_guidance_interval = 7
    eval_points = np.arange(0, T_go_guess + 
                            outer_loop_guidance_interval/2, 
                            outer_loop_guidance_interval)
    
    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)
    
    print(ode_func(0, np.concatenate((x0, v0, [m0]))))

    run_simulation(ode_func, eval_points, prob, log, log_file)

  #  return prob, log

def lunar_trajectory_final():
    log_file = "log_final.pkl"
    r0 = 1737.4e3
    mu = 4.90e12
    #x0 = np.array([r0, 0]) # CHANGE
    x0 = np.array([1727.829e3, -312.817e3])
    #v0 = np.array([0, 0]) # CHANGE
    v0 = np.array([-289.43, -1611.06])
    v_e = 3900
    m_dot = 0.42
    #m0 = 500 # CHANGE
    m0 = 317.3
    #T_go_guess = 438 #CHANGE
    T_go_guess = 2

    model = FixedThrustGuidance()

    prob = om.Problem(model)
    prob.setup()
    # Initial conditions (Also IC for run_simulation)
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = -1
    prob['r_T'] = r0 + 18.52e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # Why do we need this?
    prob.run_model()

    # temp value for log
    log = init_log(prob)

    # Integral time boundaries based on the rate we want to recalculate 
    # the outer loop guidance block.
    outer_loop_guidance_interval = 2
    eval_points = np.arange(0, T_go_guess + 
                            outer_loop_guidance_interval/2, 
                            outer_loop_guidance_interval) # LOOK AT THIS

    g0 = 9.80665
    Isp = v_e/g0
    F_thrust_max = m_dot * v_e
    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)
    
    print(ode_func(0, np.concatenate((x0, v0, [m0]))))

    run_simulation(ode_func, eval_points, prob, log, log_file)


def lunar_trajectory():
    log_filename = "log_lunar_traj.pkl"
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
    prob['sample_t'] = 0
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
    prob.run_model()

    #recorder = om.SqliteRecorder('rocket_ode.sql', record_viewer_data=False)
    #prob.add_recorder(recorder)

    g0 = 9.80665
    Isp = v_e/g0
    F_thrust_max = m_dot * v_e
    initial_time = 0
    initial_state = np.concatenate((x0, v0, [m0]))
    # temp value for log
    log = init_log(prob)

    outer_loop_interval = 7
    eval_points = np.arange(0, T_go_guess + 
                            outer_loop_interval/2, 
                            outer_loop_interval)

    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)

    run_simulation(ode_func, eval_points, prob, log, log_filename)


def lunar_trajectory_full():
    log_filename = "log_lunar_traj_full.pkl"
    r0 = 1737.4e3
    mu = 4.90e12
    x0 = np.array([r0, 0])
    v0 = np.array([0, 0])
    v_e = 3900
    m_dot = 0.42
    m0 = 500
    T_go_guess = 438

    model = FixedThrustGuidanceFull()

    prob = om.Problem(model)
    prob.setup()
    # Initial conditions
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    #prob.model.set_input_defaults('T', T_go_guess)
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 0
    prob['r_T'] = r0 + 18.52e3 # m
    prob['target_v_theta_T'] = -1685 # m/s
    #prob['target_v_theta_T'] = -1652.2326567 # m/s
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    prob.run_model()

    #recorder = om.SqliteRecorder('rocket_ode.sql', record_viewer_data=False)
    #prob.add_recorder(recorder)

    g0 = 9.80665
    Isp = v_e/g0
    F_thrust_max = m_dot * v_e
    initial_time = 0
    initial_state = np.concatenate((x0, v0, [m0]))
    # temp value for log
    log = init_log(prob)

    outer_loop_interval = 2
    eval_points = np.arange(0, -10+T_go_guess + 
                            outer_loop_interval/2, 
                            outer_loop_interval)

    guidance_func = lambda t, state: guidance_func_base(t, state, prob, log)
    ode_func = lambda t, state: rocket_ode(
        t, state, mu, Isp, F_thrust_max, guidance_func)

    #prob.run_model()
    run_simulation(ode_func, eval_points, prob, log, log_filename)


if __name__ == "__main__":
    #lunar_trajectory()
    #lunar_trajectory_final()
    #apollo_ascent()
    lunar_trajectory_full()