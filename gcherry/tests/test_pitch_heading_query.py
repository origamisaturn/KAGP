import unittest
import openmdao.api as om
from copy import deepcopy

from gcherry.cherry_guidance_refactor import PitchHeadingQuery
from gcherry.log_utils_refactor import almost_equal


def set_pitch_query_default(prob):

    r0 = 1737.4e3
    # Update t and g_eff
    a0, a1, a2 = (5.182888241994683,
                 -0.006887777058719676, 
                  9.153481725927932e-06)
    c1_radial, c2_radial = (-0.12670854928655143, 
                             0.0006085997594104416)
    c1_yaw, c2_yaw = (0, 0)

    input_dict = {'query_x': [r0, 0, 0],
                  'query_v': [0, 0, 0],
                  'query_t': 0,
                  'target_lan': 0,
                  'target_inc': 0,
                  'mu': 4.90e+12,
                  'v_e': 3900,
                  'm_dot': 0.42,
                  'm0': 500,
                  'T': 438,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'target_r_T': r0 + 18.52e3,
                  'target_r_dot_T': 0}
    
    for key, value in input_dict.items():
        prob[key] = value
    

class PitchHeadingQueryGroup(om.Group):
    def setup(self):
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])


class TestPitchQuery(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(PitchHeadingQueryGroup())
        self.prob.setup()
        set_pitch_query_default(self.prob)

    def test_case_1(self):
        # Don't really have a test case here besides making sure the
        # output looks reasonable. More useful would be testing that
        # integrated r_dot_dot matches predicted in _debug.

        # self.prob['query_t'] = 0
        self.prob.run_model()
        pitch_calc_1 = self.prob['cmd_pitch'][0]
        debug_1 = deepcopy(self.prob['_debug'])

        # Technically inaccurate due to outdated g_eff.
        self.prob['query_t'] = 229.803
        self.prob.run_model()
        pitch_calc_2 = self.prob['cmd_pitch'][0]
        debug_2 = deepcopy(self.prob['_debug'])

        print("debug1: {}".format(debug_1))
        print(pitch_calc_1)
        print("debug2: {}".format(debug_2))
        print(pitch_calc_2)

        pitch_expected_1 = 0.724753178002672
        pitch_expected_2 = 0.41136374957895405

        tol = 1e-8
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))

if __name__ == '__main__':
    unittest.main()