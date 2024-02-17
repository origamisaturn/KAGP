import krpc
import time
import math
import numpy as np
import sys
import openmdao.api as om

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', 'experiments')))
#import cherryIntMDAO
from fullGuidance import FixedThrustGuidanceBlocks

import pickle as pkl


# while True:
#     print(position())
#     print(velocity())
#     print(time())

def ksp_to_rhs_2d(coord):
    rot_mat = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
    coord_rhs = rot_mat@coord
    coord_rhs_2d = coord_rhs[:2]
    return coord_rhs_2d

def init_guidance(T_go, init_pos, init_vel):
    init_pos_rhs_2d = ksp_to_rhs_2d(init_pos)
    init_vel_rhs_2d = ksp_to_rhs_2d(init_vel)

    r0 = 1737.1e3
    mu = 4.9028e12
    x0 = init_pos_rhs_2d
    v0 = init_vel_rhs_2d # Add rotation of moon.
    Isp = 310
    g0 = 9.80665
    v_e = Isp*g0
    F_thrust_max = 15.57e3
    m_dot = F_thrust_max / v_e
    m0 = 3.967e3
    T_go = 335

    model = FixedThrustGuidanceBlocks()

    prob = om.Problem(model)
    prob.setup()

    # Initial conditions (Also IC for run_simulation)
    prob['x'] = x0
    prob['v'] = v0
    prob['t'] = 0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 9.544
    prob['r_T'] = r0 + 18.24e3 # m
    prob['target_v_theta_T'] = -1685 # m/s
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    prob.run_model()

    return prob

def update_outer_loop(prob, log, pos, vel, time):
    rhs_pos_2d = ksp_to_rhs_2d(pos)
    rhs_vel_2d = ksp_to_rhs_2d(vel)

    prob['sample_t'] = time
    prob['x'] = rhs_pos_2d
    prob['v'] = rhs_vel_2d

def query_pitch(prob, log, time):
    log_problem(prob, log)
    prob['t'] = time
    prob.run_model()
    alpha = prob['alpha']
    return np.rad2deg(alpha)[0]

def init_log(prob):
    log = {'inputs': {}, 'outputs': {}, 'state': {}}
    model = prob.model
    inputs = model.list_inputs()
    outputs = model.list_outputs()
    state_names = ['t', 'x', 'y', 'vx', 'vy', 'm']
    for var in inputs:
        var_name = var[0]
        var_val = var[1]['val']
        log['inputs'][var_name] = list()
    for var in outputs:
        var_name = var[0]
        var_val = var[1]['val']
        log['outputs'][var_name] = list()
    for var_name in state_names:
        log['state'][var_name] = list()
    return log

def log_problem(prob, log):
    inputs = log['inputs'].keys()
    outputs = log['outputs'].keys()
    for var_name in inputs:
        var_val = prob[var_name][0]
        log['inputs'][var_name].append(var_val)
    for var_name in outputs:
        var_val = prob[var_name][0]
        log['outputs'][var_name].append(var_val)

def log_state(log, pos, vel, time, mass):
    rhs_pos_2d = ksp_to_rhs_2d(pos)
    rhs_vel_2d = ksp_to_rhs_2d(vel)

    t = time
    x = rhs_pos_2d[0]
    y = rhs_pos_2d[1]
    vx = rhs_vel_2d[0]
    vy = rhs_vel_2d[1]
    m = mass
    state = {'t': t, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'm': m}
    for var_name in state:
        var_val = state[var_name]
        log['state'][var_name].append(var_val)


conn = krpc.connect(name='Sub-orbital Flight')
vessel = conn.space_center.active_vessel

ref_frame = vessel.orbit.body.non_rotating_reference_frame
mass = conn.add_stream(getattr, vessel, 'mass')
position = conn.add_stream(vessel.position, ref_frame)
velocity = conn.add_stream(vessel.velocity, ref_frame)
time = conn.add_stream(getattr, conn.space_center, 'ut')     


log_file = "log_ksp_ascent_full.pkl"
burn_time = 345
outer_loop_interval = 7
next_outer_loop_time = -1
pos = position()
vel = velocity()
prob = init_guidance(burn_time, pos, vel)
log = init_log(prob)

init_time = conn.space_center.ut
rel_time = time()-init_time

vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.engage()
vessel.control.throttle = 1

estimated_T = burn_time

while rel_time < estimated_T:
    if (burn_time > rel_time + 10 and 
        next_outer_loop_time < rel_time):
        update_outer_loop(prob, log, pos, vel, rel_time)
        next_outer_loop_time = next_outer_loop_time + outer_loop_interval
        print("Outer Loop Update")
    alpha = query_pitch(prob, log, rel_time)
    vessel.auto_pilot.target_pitch_and_heading(alpha, 90)

    with time.condition:
        time.wait()

    rel_time = time() - init_time
    pos = position()
    vel = velocity()
    m = mass()
    estimated_T = prob['T']
    print("t: {:.2f}\t alpha: {:.2f}".format(rel_time, alpha))

    log_state(log, pos, vel, rel_time, m)


vessel.control.throttle = 0
vessel.auto_pilot.disengage()

with open(log_file, 'wb') as fh:
    pkl.dump(log, fh) 
