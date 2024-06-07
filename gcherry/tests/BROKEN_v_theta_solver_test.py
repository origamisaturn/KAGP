import unittest
import numpy as np
import openmdao.api as om
from copy import deepcopy

from gcherry.cherry_guidance_refactor import VThetaSolver
from gcherry.log_utils_refactor import almost_equal

    
def set_v_theta_solver_default(prob):
    a0, a1, a2, c1, c2 = (5.182888241994683,
                        -0.006887777058719676, 
                        9.153481725927932e-06, 
                        -0.12670854928655143, 
                        0.0006085997594104416)
    r0 = 1737.4e3
    input_dict = {'sample_x': np.array([r0, 0, 0]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_r_dot_T': 0,
                  'target_r_T': r0 + 18.52e3,
                  'mu': 4.90e12,
                  'v_e': 3900,
                  'm_dot': 0.42,
                  'm0': 500,
                  'T': 438,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1,
                  'c2_radial': c2,
                  'c1_yaw': 0,
                  'c2_yaw': 0}
    
    for key, value in input_dict.items():
        prob[key] = value


class VThetaSolverGroup(om.Group):
    def setup(self):
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])

# WARNING: THESE TESTS WILL NEED TO BE UPDATED BY A COMPLETE SIMULATION
# RUN
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

        v_theta_expected = 1600
        v_theta_loss_expected = 400

        v_theta_residual = v_theta_calc - v_theta_expected
        v_theta_loss_residual = v_theta_loss_calc - v_theta_loss_expected

        tol = 1e-8
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

    def test_case_2(self):
        # Tests while in motion
        r0 = 1737.4e3
        self.prob['sample_x'] = np.array([r0+1e3, 1e3, 0])
        self.prob['sample_v'] = np.array([20, 400, 0])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_loss_calc = self.prob['v_theta_loss_T']

        v_theta_expected = 1600
        v_theta_loss_expected = 400

        v_theta_residual = v_theta_calc - v_theta_expected
        v_theta_loss_residual = v_theta_loss_calc - v_theta_loss_expected

        tol = 1e-8
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

if __name__ == '__main__':
    unittest.main()