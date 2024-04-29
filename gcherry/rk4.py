import numpy as np

def rk4(fun, tspan, y0, max_step):
    # Make sure y0 is np
    y0 = np.array(y0)

    t_res = rk4_get_t_steps(tspan, max_step)
    y_res = np.zeros((len(t_res), len(y0)))
    for i in range(len(t_res)):
        if i == 0:
            y_res[0, :] = y0
        else:
            t = t_res[i-1]
            y = y_res[i-1, :]
            h = t_res[i] - t
            y_res[i, :] = rk4_step(fun, t, y, h)
            
    return t_res, y_res

def rk4_step(fun, t, y, h):
    w1 = h*fun(t, y)
    w2 = h*fun(t + 1/2*h, y + 1/2*w1)
    w3 = h*fun(t + 1/2*h, y + 1/2*w2)
    w4 = h*fun(t + h, y + w3)
    y_next = y + 1/6 * (w1 + 2*w2 + 2*w3 + w4)
    return y_next

def rk4_get_t_steps(tspan, max_step):
    d_tspan = tspan[1] - tspan[0]
    n_steps = int(np.floor(d_tspan/max_step))
    trunc_fin_t = tspan[0] + n_steps*max_step
    adjust_fin_t = d_tspan%max_step + trunc_fin_t

    t_steps = np.linspace(tspan[0], trunc_fin_t, n_steps+1)
    if not within_tol(adjust_fin_t, trunc_fin_t, tol=1e-8):
        t_steps = np.hstack((t_steps, [adjust_fin_t]))

    return t_steps

def within_tol(val1, val2, tol=1e-8):
    if abs(val1-val2) < tol:
        return True
    else:
        return False