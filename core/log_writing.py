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