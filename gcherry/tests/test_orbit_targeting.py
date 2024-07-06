import unittest
import numpy as np
import openmdao.api as om

from gcherry.guidance_components import (
    OrbitGuidanceGroup)
from gcherry.transform import global2perifocal_rot
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

def set_orbit_targeting_scenario_2(prob):
    input_dict = {'sample_x': np.array([1481773.78741417, -855502.4950417, 301696.34387852]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_pe': 1780.0e3,
                  'target_ap': 1790.0e3,
                  'target_lan': 4.363323129985824,
                  'target_inc': 0.19198621771937624,
                  'target_argp': 0.3,
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
        theta_T_expected = 0.179561310932418

        self.prob = om.Problem(OrbitGuidanceGroup())
        self.prob.setup()
        set_orbit_targeting_scenario_1(self.prob)

        # Test from stationary start
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc, delta_theta_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'],
            self.prob['delta_theta_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        sample_x_perifocal = global2perifocal_rot(
            self.prob['target_lan'][0], 
            self.prob['target_inc'][0], 
            self.prob['target_argp'][0])@self.prob['sample_x']
        theta_0 = np.arctan2(sample_x_perifocal[1], sample_x_perifocal[0])
        self.assertTrue(almost_equal(
            delta_theta_T_calc + theta_0, theta_T_expected, tol))
        
        # Test from mid-flight
        self.prob['sample_x'] = np.array([1742947.08065715,
                                          9735.20110926976,
                                          0])
        self.prob['sample_v'] = np.array([100.943744375086,
                                          215.165643404832,
                                          0])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc, delta_theta_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'],
            self.prob['delta_theta_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        sample_x_perifocal = global2perifocal_rot(
            self.prob['target_lan'][0],
            self.prob['target_inc'][0],
            self.prob['target_argp'][0])@self.prob['sample_x']
        theta_0 = np.arctan2(sample_x_perifocal[1], sample_x_perifocal[0])        
        self.assertTrue(almost_equal(
            delta_theta_T_calc + theta_0, theta_T_expected, tol))
        
    def test_case_2(self):
        tol = 1e-2
        T_expected = 458.3449
        v_theta_T_expected = 1658.6478
        r_T_expected = 1783550.0616
        r_dot_T_expected = 4.4464
        theta_T_expected = 1.2792

        self.prob = om.Problem(OrbitGuidanceGroup())
        self.prob.setup()
        set_orbit_targeting_scenario_2(self.prob)

        # Test from stationary start
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc, delta_theta_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'],
            self.prob['delta_theta_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        sample_x_perifocal = global2perifocal_rot(
            self.prob['target_lan'][0], 
            self.prob['target_inc'][0], 
            self.prob['target_argp'][0])@self.prob['sample_x']
        theta_0 = np.arctan2(sample_x_perifocal[1], sample_x_perifocal[0])
        self.assertTrue(almost_equal(
            delta_theta_T_calc + theta_0, theta_T_expected, tol))
        
        # Test from mid-flight
        self.prob['sample_x'] = np.array([1490529.31431708,	
                                          -849577.233607873,	
                                          305913.205755655])
        self.prob['sample_v'] = np.array([178.316880048864,	
                                          141.260380569038,	
                                          78.3699021132937])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        T_calc, v_theta_T_calc, r_T_calc, r_dot_T_calc, delta_theta_T_calc = (
            self.prob['T'], self.prob['target_v_theta_T'],
            self.prob['target_r_T'], self.prob['target_r_dot_T'],
            self.prob['delta_theta_T'])
        self.assertTrue(almost_equal(T_calc - T_expected, 0, tol))
        self.assertTrue(almost_equal(
            v_theta_T_calc - v_theta_T_expected, 0, tol))
        self.assertTrue(almost_equal(r_T_calc - r_T_expected, 0, tol))
        self.assertTrue(almost_equal(
            r_dot_T_calc - r_dot_T_expected, 0, tol))
        sample_x_perifocal = global2perifocal_rot(
            self.prob['target_lan'][0],
            self.prob['target_inc'][0],
            self.prob['target_argp'][0])@self.prob['sample_x']
        theta_0 = np.arctan2(sample_x_perifocal[1], sample_x_perifocal[0])        
        self.assertTrue(almost_equal(
            delta_theta_T_calc + theta_0, theta_T_expected, tol))

if __name__ == '__main__':
    unittest.main()
