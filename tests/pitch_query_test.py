import unittest
import numpy as np
import math
import openmdao.api as om
from copy import deepcopy

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', 'core')))
from cherry_guidance_refactor import PitchQuery

def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol
    

def set_pitch_query_default(prob):

    # Update t and g_eff
    a0, a1, a2, c1, c2 = (5.182888241994683,
                          -0.006887777058719676, 
                          9.153481725927932e-06, 
                          -0.12670854928655143, 
                          0.0006085997594104416)
    input_dict = {'t': 0,
                  'T': 438,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1': c1,
                  'c2': c2,
                  'g_eff': 0,
                  'm0': 500,
                  'm_dot': 0.42,
                  'v_e': 3900}
    
    for key, value in input_dict.items():
        prob[key] = value
    

class PitchQueryGroup(om.Group):
    def setup(self):
        self.add_subsystem('pitch_query', PitchQuery(), promotes=['*'])


class TestPitchQuery(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(PitchQueryGroup())
        self.prob.setup()
        set_pitch_query_default(self.prob)

    def test_case_1(self):
        # Don't really have a test case here besides making sure the
        # output looks reasonable. More useful would be testing that
        # integrated r_dot_dot matches predicted in _debug.
        self.prob['g_eff'] = -1.62329124

        self.prob['t'] = 0
        self.prob.run_model()
        alpha_calc_1 = self.prob['alpha'][0]
        debug_1 = deepcopy(self.prob['_debug'])

        # Technically inaccurate due to outdated g_eff.
        self.prob['t'] = 229.803
        self.prob.run_model()
        alpha_calc_2 = self.prob['alpha'][0]
        debug_2 = deepcopy(self.prob['_debug'])

        print("debug1: {}".format(debug_1))
        print(alpha_calc_1)
        print("debug2: {}".format(debug_2))
        print(alpha_calc_2)

        alpha_expected_1 = 0.724753178002672
        alpha_expected_2 = 0.41136374957895405

        tol = 1e-8
        self.assertTrue(almost_equal(alpha_calc_1, alpha_expected_1, tol))
        self.assertTrue(almost_equal(alpha_calc_2, alpha_expected_2, tol))

if __name__ == '__main__':
    unittest.main()