import unittest
import numpy as np
import openmdao.api as om
from copy import deepcopy

from gcherry.cherry_guidance_refactor import PitchHeadingQuery
from gcherry.log_utils_refactor import almost_equal


# Takeoff from lunar surface along equator to a position with 0 r_dot.
# Only radial guidance, no yaw.
def set_pitch_query_scenario_1(prob):
    a0, a1, a2 = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06,
    )
    c1_radial, c2_radial = (
        -0.325367567512005,
        0.00156271732299607
    )
    c1_yaw, c2_yaw = (0, 0)
    # Lunar radius
    r0 = 1737.4e3
    input_dict = {'query_x': [r0, 0, 0],
                  'query_v': [0, 0, 0],
                  'query_t': 0,
                  'target_lan': 0,
                  'target_inc': 0,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
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
        set_pitch_query_scenario_1(self.prob)

    def test_case_1(self):
        tol = 1e-8

        pitch_expected_1 = 0.724753178002672
        heading_expected_1 = np.deg2rad(90)
        self.prob['query_t'] = 0
        self.prob.run_model()
        pitch_calc_1 = self.prob['cmd_pitch'][0]
        heading_calc_1 = self.prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.41136374957895405
        heading_expected_2 = np.deg2rad(90)
        self.prob['query_t'] = 229.803
        self.prob['query_x'] = 
        self.prob['query_v'] = 
        self.prob.run_model()
        pitch_calc_2 = self.prob['cmd_pitch'][0]
        heading_calc_2 = self.prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))

    def test_case_2(self):
        tol = 1e-8

        pitch_expected_1 = 0.724753178002672
        heading_expected_1 = np.deg2rad(90)
        self.prob['query_t'] = 0
        self.prob.run_model()
        pitch_calc_1 = self.prob['cmd_pitch'][0]
        heading_calc_1 = self.prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.41136374957895405
        heading_expected_2 = np.deg2rad(90)
        self.prob['query_t'] = 229.803
        self.prob['query_x'] = 
        self.prob['query_v'] = 
        self.prob.run_model()
        pitch_calc_2 = self.prob['cmd_pitch'][0]
        heading_calc_2 = self.prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))


if __name__ == '__main__':
    unittest.main()