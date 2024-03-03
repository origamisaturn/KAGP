import unittest
import numpy as np
import math
import openmdao.api as om
from copy import deepcopy

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', 'core')))
from cherry_guidance_refactor import TimeToGo

def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol
    
def set_time_to_go_default(prob):
    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([r0, 0]),
                  'sample_v': np.array([0, 0]),
                  'sample_t': 0,
                  'target_v_theta_T': 1600,
                  'v_e': 3900,
                  'm_dot': 0.42,
                  'm0': 500,
                  'v_theta_T': 1400,
                  'v_theta_loss_T': 400}
    
    for key, value in input_dict.items():
        prob[key] = value

class TimeToGoGroup(om.Group):
    def setup(self):
        self.add_subsystem('time_to_go', TimeToGo(), promotes=['*'])

class TestVThetaSolver(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(TimeToGoGroup())
        self.prob.setup()
        set_time_to_go_default(self.prob)

    def test_case_1(self):
        self.prob.run_model()
        T_calc = self.prob['T']

        T_expected = 438

        T_residual = T_calc - T_expected

        tol = 1e-8
        self.assertTrue(almost_equal(T_residual, 0, tol))

if __name__ == '__main__':
    unittest.main()