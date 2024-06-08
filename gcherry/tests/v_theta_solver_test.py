import unittest
import numpy as np
import math
import openmdao.api as om
from copy import deepcopy

from gcherry.cherry_guidance_refactor import VThetaSolver

def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol
    
def set_v_theta_solver_default(prob):
    a0, a1, a2, c1_radial, c2_radial = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06,
        -0.325367567512005,
        0.00156271732299607)

    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([r0, 0, 0]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_r_dot_T': 0,
                  'target_r_T': 1785.0e3,
                  'T': 438,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500}
    
    for key, value in input_dict.items():
        prob[key] = value


class VThetaSolverGroup(om.Group):
    def setup(self):
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])


class TestVThetaSolver(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(VThetaSolverGroup())
        self.prob.setup()
        set_v_theta_solver_default(self.prob)
    
    def test_case_1(self):
        # Test from stationary start.
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_loss_calc = self.prob['v_theta_loss_T']

        v_theta_expected = 1549.78024878931
        v_theta_loss_expected = 240.138817033841

        v_theta_residual = v_theta_calc - v_theta_expected
        v_theta_loss_residual = v_theta_loss_calc - v_theta_loss_expected

        tol = 1e-3
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

    def test_case_2(self):
        # Tests while in motion
        self.prob['sample_x'] = np.array([1743371.45973407,
                                          9064.77377033883,
                                          0])
        self.prob['sample_v'] = np.array([108.553696158295,
                                          204.538775905435,
                                          0])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        # v_theta_loss_calc = self.prob['v_theta_loss_T']

        v_theta_expected = 1549.78024878931
        # v_theta_loss_expected = 240.138817033841

        v_theta_residual = v_theta_calc - v_theta_expected
        # v_theta_loss_residual = v_theta_loss_calc - v_theta_loss_expected

        tol = 1e-3
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        # self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

    def test_case_3:
        ...

if __name__ == '__main__':
    unittest.main()