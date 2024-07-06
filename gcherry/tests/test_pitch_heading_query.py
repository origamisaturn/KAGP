import unittest
import numpy as np
import openmdao.api as om
from copy import deepcopy

from gcherry.guidance_components import PitchHeadingQuery
from gcherry.log_utils_refactor import almost_equal


# See test_debug_ascent_1_scenario_1.yaml
def set_pitch_query_scenario_1(prob):
    a0, a1, a2 = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06
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
                  'target_r_T': 1785e+3,
                  'target_r_dot_T': 0}
    for key, value in input_dict.items():
        prob[key] = value

def set_pitch_query_scenario_2(prob):
    a0, a1, a2 = (
        5.4192248771367,
        -0.00754333097672137,
        1.04999964966261E-05
    )
    c1_radial, c2_radial = (
        -0.236081903268302,
        0.00109625350418313
    )
    c1_yaw, c2_yaw = (
        -0.144528539161392,
        0.000644824881978622
    )
    # Lunar radius
    r0 = 1737.4e3
    input_dict = {'query_x': [
                    1481773.78741417, 
                    -855502.4950417, 
                    301696.34387852],
                  'query_v': [0, 0, 0],
                  'query_t': 0,
                  'target_lan': 4.363323129985824,
                  'target_inc': 0.19198621771937624,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500,
                  'T': 470,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'target_r_T': 1785e+3,
                  'target_r_dot_T': 20.0}
    for key, value in input_dict.items():
        prob[key] = value
    

class PitchHeadingQueryGroup(om.Group):
    def setup(self):
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])


class TestPitchQuery(unittest.TestCase):
    def test_case_1(self):
        prob = om.Problem(PitchHeadingQueryGroup())
        prob.setup()
        set_pitch_query_scenario_1(prob)
        tol = 1e-8

        pitch_expected_1 = 1.18447886794603
        heading_expected_1 = np.deg2rad(90)
        prob['query_t'] = 0
        prob.run_model()
        pitch_calc_1 = prob['cmd_pitch'][0]
        heading_calc_1 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.728365591911019
        heading_expected_2 = np.deg2rad(90)
        prob['query_t'] = 100
        prob['query_x'] = [
            1743371.45973407, 
            9064.77377033883, 
            5.5505786640278E-13
        ]
        prob['query_v'] = [
            108.553696158295, 
            204.538775905435, 
            1.25244257082427E-14
        ]
        prob.run_model()
        pitch_calc_2 = prob['cmd_pitch'][0]
        heading_calc_2 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))

    def test_case_2(self):
        prob = om.Problem(PitchHeadingQueryGroup())
        prob.setup()
        set_pitch_query_scenario_2(prob)
        tol = 1e-8

        pitch_expected_1 = 1.02193090008912
        heading_expected_1 = 1.12721950511188
        prob['query_t'] = 0
        prob.run_model()
        pitch_calc_1 = prob['cmd_pitch'][0]
        heading_calc_1 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.689203268175016
        heading_expected_2 = 1.39664493990468
        prob['query_t'] = 100
        prob['query_x'] = [
            1490528.02411845,
            -849036.637175544,
            305753.440395322
        ]
        prob['query_v'] = [
            177.904433951903,
            149.952620578308,
            75.5017709619082
        ]
        prob.run_model()
        pitch_calc_2 = prob['cmd_pitch'][0]
        heading_calc_2 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))


if __name__ == '__main__':
    unittest.main()