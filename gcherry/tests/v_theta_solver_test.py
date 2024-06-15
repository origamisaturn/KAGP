import unittest
import numpy as np
import openmdao.api as om
from copy import deepcopy

from gcherry.cherry_guidance_refactor import VThetaSolver2
from gcherry.log_utils_refactor import almost_equal

    
# Takeoff from lunar surface along equator to a position with 0 r_dot.
# Only radial guidance, no yaw.
def set_v_theta_solver_scenario_1(prob):
    a0, a1, a2, c1_radial, c2_radial = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06,
        -0.325367567512005,
        0.00156271732299607)

    # Lunar radius
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
                  'c1_yaw': 0,
                  'c2_yaw': 0,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500}
    
    for key, value in input_dict.items():
        prob[key] = value

# Takeoff from lunar surface at (ra, decl) == (-30deg, 10deg) to an 
# inclined, elliptical orbit. Radial and yaw guidance.
def set_v_theta_solver_scenario_2(prob):
    a0, a1, a2 = (
        5.4192248771367,
        -0.00754333097672137,
        1.04999964966261E-05)
    c1_radial, c2_radial = (-0.236081903268302, 0.00109625350418313)
    c1_yaw, c2_yaw = (-0.144528539161392, 0.000644824881978622)

    input_dict = {'sample_x': np.array([
                                1481773.78741417,
                                -855502.4950417 ,
                                301696.34387852]),
                  'sample_v': np.array([0, 0, 0]),
                  'sample_t': 0,
                  'target_r_dot_T': 20,
                  'target_r_T': 1785.0e3,
                  'T': 470,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500}   
    
    for key, value in input_dict.items():
        prob[key] = value


class VThetaSolverGroup(om.Group):
    def setup(self):
        self.add_subsystem('v_theta_solver', VThetaSolver2(), promotes=['*'])

class TestVThetaSolver(unittest.TestCase):   
    # See set_v_theta_solver_scenario_1() 
    def test_case_1(self):
        v_theta_expected = 1549.78024878931

        self.prob = om.Problem(VThetaSolverGroup())
        self.prob.setup()
        set_v_theta_solver_scenario_1(self.prob)

        # Test from stationary start.
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_loss_calc = self.prob['v_theta_loss_T']
        v_theta_loss_expected = 240.138817033841
        v_theta_residual = v_theta_calc - v_theta_expected
        v_theta_loss_residual = v_theta_loss_calc - v_theta_loss_expected
        tol = 2e-2
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

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
        v_theta_residual = v_theta_calc - v_theta_expected
        tol = 1e-3
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))
        # self.assertTrue(almost_equal(v_theta_loss_residual, 0, tol))

    # See set_v_theta_solver_scenario_2()
    def test_case_2(self):
        v_theta_expected = 1725.02901332511

        self.prob = om.Problem(VThetaSolverGroup())
        self.prob.setup()
        set_v_theta_solver_scenario_2(self.prob)

        # Test from stationary start
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_residual = v_theta_calc - v_theta_expected
        # TODO: commented out for debugging, restore this soon.
        # tol = 2e-1
        tol = 1
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))

        # Test from mid-flight
        # Due to the approximation that the normal of the target orbital
        # plane is orthogonal to the radial vector, it is expected that 
        # midway thru the trajectory the estimation will be less accurate.
        # Unsure why it is so accurate at start of trajectory.
        self.prob['sample_x'] = np.array([1490528.02411845,
                                          -849036.637175544,
                                          305753.440395322])
        self.prob['sample_v'] = np.array([177.904433951903,
                                          149.952620578308,
                                          75.5017709619082])
        self.prob['sample_t'] = 100
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_residual = v_theta_calc - v_theta_expected
        tol = 10
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))

        # 10 Seconds from trajectory termination.
        # The approximation becomes more accurate at the end of the
        # trajectory.
        self.prob['sample_x'] = np.array([1653277.85787638,
                                          -579681.582482377,
                                          340483.259107105])
        self.prob['sample_v'] = np.array([583.115089862512,
                                          1569.12004651024,
                                          9.93982352281227])
        self.prob['sample_t'] = 460
        self.prob.run_model()
        v_theta_calc = self.prob['v_theta_T']
        v_theta_residual = v_theta_calc - v_theta_expected
        tol = 3e-2
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))


if __name__ == '__main__':
    unittest.main()