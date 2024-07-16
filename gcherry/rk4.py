import numpy as np

# callback added to give program opportunity to calculate outer loop every step.
def rk4(fun, tspan, y0, max_step, callback=None):
    """ Runge-kutta fourth-order integrator.
    
    Args:
        fun: Derivative function, with arguments t (float) and state (1-D 
            array of state at time t). Returns derivative of state in 1-D
            array, with length equal to that of state.
        tspan: 2-element list consisting of start t and end t, in that 
            order.
        y0: Initial state, 1-D array.
        max_step: Maximum change in t for each step.
        callback: Function called at the end of every rk4 step. Arguments
            are t (float) and state (1-D array of state at time t), 
            representing the t and state just calculated.
        
    Returns:
        (t_res, y_res): t_res is a 1-D numpy array consisting of 
        the time after each step of rk4. y_res is a 2-D numpy
        array where the rows are the elements of the state, and
        the columns are the state at different times, 
        corresponding to t_res. 
        
        The final result is given by y_res[:, -1]. 
        
    """
    # Make sure y0 is np
    y0 = np.array(y0)

    # hopefully calling a useless function is more performant than checking
    # if callback is None every integration step
    if callback==None:
        callback = lambda t, y: None

    t_res = _rk4_get_t_steps(tspan, max_step)
    y_res = np.zeros((len(y0), len(t_res)))
    for i in range(len(t_res)):
        if i == 0:
            y_res[:, 0] = y0
        else:
            t = t_res[i-1]
            y = y_res[:, i-1]
            h = t_res[i] - t
            y_res[:, i] = rk4_step(fun, t, y, h)

            t_new = t_res[i]
            y_new = y_res[:, i]
            # add a test for the callback method
            callback(t_new, y_new)

    return t_res, y_res

def rk4_step(fun, t, y, h):
    w1 = h*fun(t, y)
    w2 = h*fun(t + 1/2*h, y + 1/2*w1)
    w3 = h*fun(t + 1/2*h, y + 1/2*w2)
    w4 = h*fun(t + h, y + w3)
    y_next = y + 1/6 * (w1 + 2*w2 + 2*w3 + w4)
    return y_next

def _rk4_get_t_steps(tspan, max_step):
    d_tspan = tspan[1] - tspan[0]
    n_steps = int(np.floor(d_tspan/max_step))
    trunc_fin_t = tspan[0] + n_steps*max_step
    # adjust_fin_t = d_tspan%max_step + trunc_fin_t
    true_fin_t = tspan[1]

    t_steps = np.linspace(tspan[0], trunc_fin_t, n_steps+1)
    if not within_tol(true_fin_t, trunc_fin_t, tol=1e-8):
        t_steps = np.hstack((t_steps, true_fin_t))
    else:
        t_steps[-1] = true_fin_t

    # d_tspan = tspan[1] - tspan[0]
    # n_steps = int(np.ceil(d_tspan/max_step))
    # t_steps = np.linspace(tspan[0], tspan[1], n_steps+1)
    return t_steps

def within_tol(val1, val2, tol=1e-8):
    if abs(val1-val2) < tol:
        return True
    else:
        return False