def init_log(prob):
    """ Initializes a log dictionary. 
    
    Args:
        prob: An OpenMDAO problem.
    
    Returns:
        A dictionary with keys ['inputs, 'outputs', 'state']. The
        values of state are 1D lists of variables. The values of inputs
        and outputs are lists of variables, whose dimensions are based
        on prob.model.list_inputs() and .list_outputs(). The first 
        dimension is the index of entry and the remaining dimensions 
        correspond to the dimensions of the variables.

        If a variable is a dictionary, its elements should be lists.
    """
    log = {'inputs': {}, 'outputs': {}, 'state': {}}
    model = prob.model
    inputs = model.list_inputs()
    outputs = model.list_outputs()
    state_names = ['t', 'x', 'y', 'vx', 'vy', 'm']

    input_data = [(x[0], x[1]['val']) for x in inputs]
    output_data = [(x[0], x[1]['val']) for x in outputs]

    for var_name, var_val in input_data:
        log['inputs'][var_name] = list()

    for var_name, var_val in output_data:
        # For _debug output
        if type(var_val) == type(dict()):
            log['outputs'][var_name] = {}
            for key in var_val:
                log['outputs'][var_name][key] = []
        else:
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
        if type(prob[var_name]) == type(dict()):
            for key in prob[var_name]:
                log['outputs'][var_name][key].append(var_val)
        else:
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