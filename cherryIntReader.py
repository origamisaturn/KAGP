import openmdao.api as om
import numpy as np
import math
import matplotlib.pyplot as plt

cr = om.CaseReader("rocket_ode.sql")
problem_cases = cr.get_cases('problem', recurse=False)
test_vals = []
t_vals = []
for case in problem_cases:
    test_vals.append(case['alpha'][0])
    t_vals.append(case['t'])
plt.figure()
plt.plot(t_vals, test_vals)
plt.figure()
plt.plot(t_vals)
plt.show()