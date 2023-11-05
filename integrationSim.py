def run_simulation(ode_func, eval_points, prob, log, log_file):
    # please encode arg information in ode_func when passec
    # somehow align eval_points and outer loop guidance rate.
    # extract initial state from problem
    x0 = prob['x']
    v0 = prob['v']
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
        res = solve_ivp(ode_func, t_span, initial_state)

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
        prob['x'] = x
        prob['v'] = v

        # Set up next problem loop.
        final_state = res.y[:, -1]
        initial_state = final_state

    with open(log_file, 'wb') as fh:
        pkl.dump(log, fh) 
