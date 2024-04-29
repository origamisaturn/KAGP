import numpy as np
from scipy.integrate import solve_ivp
import pickle as pkl

def guidance_func_base(t, state, prob, log):
    """ Given t and state, will compute desired thrust and thrust angle.
        Essentially a wrapper for the OpenMDAO guidance problem.
        Inputs:
            t       double  [s] Simulation time
            state   list    Equivalent 
            prob    om.Problem  Contains FixedThrustGuidance model.
            log     dict    XX
        Outputs:
            thrust_mag      double  In interval [0,1], indicates
                            percentage of max_thrust.
            thrust_angle    double  [rad.]  In interval [-pi,
                            pi]. 0 degrees indicates eastward, incre
                            asing angle counter clockwise.
                            
        Update outer guidance every second, inner guidance can be 
        sampled infinitely.
        Take in t, state, put into problem inputs. t goes both into
        sample_t and just t. call run_model(). Extract T. If t > T, then
        send thrust-off command and hold pitch.
        run_model() should model outer and inner loop.
    """
    # IMPORTANT, this should not be obscured.
    attitude_hold_span = 0 #s
    T = prob['T']
    if t > T:
        thrust_mag = 0
        thrust_angle = prob['alpha']
    elif t >= T - attitude_hold_span:
        thrust_mag = 1
        thrust_angle = prob['alpha']
        # Return model values without recalculating
    else:
        x = state[0:2]
        v = state[2:4]
        # Updating this should not cause an OuterLoopRadialControl update
        # without updating sample_t. It would be nice to call some function
        # that updates OuterLoopRadialControl for me.
        # This does however update PerfectPitchQuery.
        # Also unfortunately updates VThetaTest, so log results are erroneous.
        prob['x'] = x
        prob['v'] = v
        prob['t'] = t
        # There is no estimator for m since it is perfectly known already.
        prob.run_model()
        log_problem(prob, log)
        thrust_mag = 1
        thrust_angle = prob['alpha']
    
    return thrust_mag, thrust_angle


def run_simulation_rk4(ode_func, eval_points, prob, log, log_file):
    # please encode arg information in ode_func when passec
    # somehow align eval_points and outer loop guidance rate.
    # extract initial state from problem
    x0 = prob['sample_x']
    v0 = prob['sample_v']
    m0 = prob['m0']
    initial_state = np.concatenate((x0, v0, m0))

    # Evaluate every time interval in eval_points
    N = len(eval_points)
    for i in range(N-1):
        # Select time interval
        t_span = eval_points[i:i+2] 

        # Print progress
        print("t: {}".format(t_span[0]))

        if t_span[0] > 260:
            print("stop")

        # Calculate time step
        res = solve_ivp(ode_func, t_span, initial_state, atol=1e-9, rtol=1e-9)

        # Log time-step result and avoid duplicating end of previous 
        # and start of current res.y
        if i == N-2:
            log_res(res, log, omit_final=False)
        else:
            log_res(res, log, omit_final=True)

        # Update guidance loop
        # sample_t, x, v is changed only here. Otherwise the openMDAO problem
        # recomputes the outer loop.
        # Should add instruction of this in FixedThrustGuidance
        # And perhaps make this a function called "Update OuterLoopData"
        # or something.
        x = res.y[:2, -1]
        v = res.y[2:4, -1]
        prob['sample_t'] = t_span[1]
        prob['sample_x'] = x
        prob['sample_v'] = v

        # Set up next problem loop.
        final_state = res.y[:, -1]
        initial_state = final_state
        print(final_state)

    with open(log_file, 'wb') as fh:
        pkl.dump(log, fh) 

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
    model = prob.model
    inputs = log['inputs'].keys()
    outputs = log['outputs'].keys()
    for var_name in inputs:
        var_val = prob[var_name][0]
        log['inputs'][var_name].append(var_val)
    for var_name in outputs:
        var_val = prob[var_name][0]
        log['outputs'][var_name].append(var_val)

def log_res(res, log, omit_final=False):
    """
    unlike log_problem, log_res is called outside of the ode_func 
    as the ode evaluations may not accurately represent the state
    at the given time.
    """
    t = res.t
    x = res.y[0, :]
    y = res.y[1, :]
    vx = res.y[2, :]
    vy = res.y[3, :]
    m = res.y[4, :]
    state = {'t': t, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'm': m}
    for var_name in state:
        if omit_final == False:
            var_val = state[var_name]
        else:
            var_val = state[var_name][:-1]
        log['state'][var_name].extend(var_val)
