import unittest
import numpy as np
import openmdao.api as om
from gcherry.cherry_guidance_refactor import (
    OrbitGuidanceGroup)
from gcherry.log_utils_refactor import almost_equal

# Takeoff from lunar surface along equator to a position with 0 r_dot.
# Only radial guidance, no yaw.    
def set_orbit_targeting_scenario_1(prob):
    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([r0, 0, 0]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_pe': 1785.0e3,
                  'target_ap': 1785.0e3,
                  'target_lan': 0.0,
                  'target_inc': 0.0,
                  'target_argp': 0.0,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500}
    
    for key, value in input_dict.items():
        prob[key] = value

class TestOrbitTargetingGroup(unittest.TestCase):
    def test_case_1(self):
        tol = 1e-6
        T_expected = 457.074553487163
        v_theta_T_expected = 1657.307052620807
        r_T_expected = 1785000
        r_dot_T_expected = 0

        self.prob = om.Problem(OrbitGuidanceGroup())
        self.prob.setup()
        set_orbit_targeting_scenario_1(self.prob)

        # Test from stationary start
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        
        # Test from mid-flight
        self.prob['sample_x'] = np.array([1742947.08065715,
                                          9735.20110926976,
                                          0])
        self.prob['sample_v'] = np.array([100.943744375086,
                                          215.165643404832,
                                          0])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        
    def test_case_2(self):
        

if __name__ == '__main__':
    unittest.main()