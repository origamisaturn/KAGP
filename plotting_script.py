import sys, os
sys.path.append(os.path.abspath('core'))

from log_utils import *

log_file = "script_sim_test.pkl"
with open(log_file, 'rb') as fh:
    log = pkl.load(fh)
plot_state(log)
plot_derived_state(log)
plot_problem_inputs(log)
plot_problem_outputs(log)
r0 = 1737.4e3
expected_fin_r = 1752953.568741297
fin_r_error = get_radius(log)[-1] - expected_fin_r
fin_r_dot = get_r_dot(log)[-1] # 8.344093818732054
fin_v_theta = get_v_theta(log)[-1] # 1685.2769140231621
print("Final r err: {}".format(fin_r_error))
print("Final r_dot: {}".format(fin_r_dot))
print("Final v_theta: {}".format(fin_v_theta))
plt.show()
r_dot_dot = get_r_dot_dot(log)
t = log['state']['t']
print("t")