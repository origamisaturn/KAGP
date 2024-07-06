import unittest
import numpy as np
import openmdao.api as om

from gcherry.guidance_components import (
    OuterLoopGroupRefactor)
from gcherry.log_utils import almost_equal


# Takeoff from lunar surface along equator to a position with 0 r_dot.
# Only radial guidance, no yaw.    
def _set_outer_loop_component_scenario1(prob):
    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([r0, 0, 0]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_r_dot_T': 0,
                  'target_r_T': 1785.0e3,
                  'target_lan': 0.0,
                  'target_inc': 0.0,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500,
                  'target_v_theta_T': 1549.78024878931}
    
    for key, value in input_dict.items():
        prob[key] = value

# Takeoff from lunar surface at (ra, decl) == (-30deg, 10deg) to an 
# inclined, elliptical orbit. Radial and yaw guidance.
def _set_outer_loop_component_scenario2(prob):
    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([
                                1481773.78741417,
                                -855502.4950417 ,
                                301696.34387852]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_r_dot_T': 20,
                  'target_r_T': 1785.0e3,
                  'target_lan': 4.363323129985824,
                  'target_inc': 0.19198621771937624,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500,
                  'target_v_theta_T': 1725.02901332511}
    
    for key, value in input_dict.items():
        prob[key] = value


class TestOuterLoopComponent(unittest.TestCase):
    # See _set_outer_loop_component_scenario1()
    def test_case_1(self):
        T_expected = 438

        self.prob = om.Problem(OuterLoopGroupRefactor())
        self.prob.setup()
        _set_outer_loop_component_scenario1(self.prob)

        # Test from stationary start
        self.prob.run_model()
        T_calc = self.prob['T']
        T_residual = T_calc - T_expected
        tol = 1e-3
        self.assertTrue(almost_equal(T_residual, 0, tol))

        # Test from mid-flight
        #self.prob.model.time_to_go.is_first_entry = True
        self.prob['sample_x'] = np.array([1743371.45973407,
                                          9064.77377033883,
                                          0])
        self.prob['sample_v'] = np.array([108.553696158295,
                                          204.538775905435,
                                          0])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        T_calc = self.prob['T']
        T_residual = T_calc - T_expected
        tol = 1e-3
        self.assertTrue(almost_equal(T_residual, 0, tol))

        # See set_time_to_go_scenario2()
    def test_case_2(self):
        T_expected = 470

        self.prob = om.Problem(OuterLoopGroupRefactor())
        self.prob.setup()
        _set_outer_loop_component_scenario2(self.prob)

        # Test from stationary start
        self.prob.run_model()
        T_calc = self.prob['T']
        T_residual = T_calc - T_expected
        tol = 2e-2
        self.assertTrue(almost_equal(T_residual, 0, tol))

        # Test from mid-flight
        self.prob['sample_x'] = np.array([1490528.02411845,
                                          -849036.637175544,
                                          305753.440395322])
        self.prob['sample_v'] = np.array([177.904433951903,
                                          149.952620578308,
                                          75.5017709619082])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        T_calc = self.prob['T']
        T_residual = T_calc - T_expected
        # Less accurate due to approximation used by v_theta_solver
        tol = 2
        self.assertTrue(almost_equal(T_residual, 0, tol))

        # 10 Seconds from trajectory termination.
        self.prob['sample_x'] = np.array([1653277.85787638,
                                          -579681.582482377,
                                          340483.259107105])
        self.prob['sample_v'] = np.array([583.115089862512,
                                          1569.12004651024,
                                          9.93982352281227])
        self.prob['sample_t'] = 460
        self.prob.run_model()
        T_calc = self.prob['T']
        T_residual = T_calc - T_expected
        # v_theta_solver becomes more accurate nearing the end of the
        # trajectory
        tol = 4e-3
        self.assertTrue(almost_equal(T_residual, 0, tol))   

if __name__ == '__main__':
    unittest.main()