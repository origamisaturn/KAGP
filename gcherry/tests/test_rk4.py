import numpy as np

from kagp.rk4 import rk4, within_tol

# Following tests based on Ordinary Differential Equations, Tenenbaum and
# Pollard, p.656 - 658


def sample_ode(x, y):
    return x**2 + y


# Nominal
def test_rk4_1():
    y0 = [1]
    xspan = [0, 0.2]
    h = 0.1

    t_res, y_res = rk4(sample_ode, xspan, y0, h)

    assert t_res[0] == xspan[0] and t_res[-1] == xspan[-1]
    assert y_res[0, 0] == y0
    assert within_tol(y_res[0, -1], 1.2242081, tol=1e-7)


# Tests t_res behavior with unequal step size
def test_rk4_2():
    y0 = [1]
    xspan = [0, 0.2]
    # Step size will produce len(t_res) == 3
    h = 0.095

    t_res, y_res = rk4(sample_ode, xspan, y0, h)

    assert t_res[0] == xspan[0] and t_res[-1] == xspan[-1]
    assert within_tol(t_res[1], h)
    assert within_tol(t_res[2], 2*h)

    assert y_res[0, 0] == y0
    assert within_tol(y_res[0, -1], 1.2242081, tol=1e-7)


# Tests single step
def test_rk4_3():
    y0 = [1]
    xspan = [0, 0.2]
    # Greater than necessary max step size.
    h = 0.3

    t_res, y_res = rk4(sample_ode, xspan, y0, h)

    assert len(t_res) == 2
    assert t_res[0] == xspan[0] and t_res[-1] == xspan[-1]
    assert y_res[0, 0] == y0

    assert within_tol(y_res[0, -1], 1.2242067, tol=1e-7)


# Following tests based on simple frictionless 2-dimensional ballistics.

def projectile_ode(t, state):
    # State: [x_dot, y_dot, x, y]
    g = 9.81 # m/s**2
    x_dot = state[0]
    y_dot = state[1]
    x_dot_dot = 0
    y_dot_dot = -g
    state_dot = np.array([x_dot_dot, y_dot_dot, x_dot, y_dot])
    return state_dot


def projectile_analytical(t, x_dot_0, y_dot_0, x0, y0):
    g = 9.81 # m/s**2
    y = -1/2*g*t**2 + y_dot_0*t + y0
    x = x_dot_0*t + x0
    return [x, y]


def test_rk4_4():
    # Based on projectile under constant acceleration
    x0 = 0
    y0 = 0
    x_dot_0 = 8.175
    y_dot_0 = 24.525
    t_apex = 2.5
    t_final = 5

    # Analytical position calc
    expected_apex_pos = projectile_analytical(t_apex, x_dot_0, y_dot_0, x0, y0)
    x_expected_apex = expected_apex_pos[0]
    y_expected_apex = expected_apex_pos[1]

    expected_final_pos = projectile_analytical(t_final, x_dot_0, y_dot_0, x0, y0)
    x_expected_final = expected_final_pos[0]
    y_expected_final = expected_final_pos[1]

    # Integrated position calc
    state_0 = [x_dot_0, y_dot_0, x0, y0]
    t_span_apex = [0, t_apex]
    h = 0.5
    _, y_res_apex = rk4(projectile_ode, t_span_apex, state_0, h)

    t_span_final = [0, t_final]
    _, y_res_final = rk4(projectile_ode, t_span_final, state_0, h)

    # Comparing results
    tol = 1e-8
    assert within_tol(x_expected_apex, y_res_apex[2, -1], tol=tol)
    assert within_tol(y_expected_apex, y_res_apex[3, -1], tol=tol)
    assert within_tol(x_expected_final, y_res_final[2, -1], tol=tol)
    assert within_tol(y_expected_final, y_res_final[3, -1], tol=tol)


# Uses pytest instead of unittest
if __name__ == '__main__':
    test_rk4_1()
    test_rk4_2()
    test_rk4_3()
    test_rk4_4()
