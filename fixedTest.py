import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt
from cherryIntMDAO import FixedThrustGuidance, FixedThrustGuidanceFull
from cherryInt import rocket_ode
from scipy.integrate import solve_ivp
import pickle as pkl


r0 = 1737.4e3
mu = 4.90e12
x0 = np.array([r0, 0])
v0 = np.array([0, 0])
v_e = 3900
m_dot = 0.42
m0 = 500
T_go_guess = 449.432

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
# Physical constants 
prob['mu'] = mu
prob['v_e'] = v_e
prob['m_dot'] = m_dot
prob['m0'] = m0
prob.run_model()