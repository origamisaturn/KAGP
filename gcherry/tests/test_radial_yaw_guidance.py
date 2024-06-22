import unittest
import numpy as np
import math
import openmdao.api as om

from gcherry.cherry_guidance_refactor import RadialYawGuidance
from gcherry.log_utils_refactor import almost_equal 

def set_radial_yaw_guidance_default(prob):
    """ Sets default input values for RadialYawGuidance.
    
    Args:
        prob: openmdao.api.Problem containing only a RadialYawGuidance() explicit
            component.
    
    """
    r0 = 1737.4e3

    input_dict = {
        'sample_x': [r0, 0, 0],
        'sample_v': [0, 0, 0],
        'sample_t': 0,
        'target_r_T': r0 + 18.52e3,
        'target_r_dot_T': 0,
        'target_lan': 0,
        'target_inc': 0,
        'v_e': 3900,
        'm_dot': 0.42,
        'm0': 500,
        'T': 438}
    
    for key, value in input_dict.items():
        prob[key] = value

def get_radial_guidance_coefficients(prob):
    """ Convenience function for getting RadialYawGuidance() output.
    
    Args:
        prob: openmdao.api.Problem containing only a RadialYawGuidance() explicit
            component.
    
    Returns:
        (a0, a1, a2, c1, c2): Coefficients which describe the commanded
          radial acceleration of the spacecraft over time.
    
    """
    coefficient_list = []
    coefficient_keys = ['a0', 'a1', 'a2', 'c1_radial', 'c2_radial']
    for key in coefficient_keys:
        coefficient_list.append(prob[key])
    return tuple(coefficient_list)

def calculate_final_radial_state(a0, a1, a2, c1_radial, c2_radial, Tgo, r0, r_dot_0):
    """ Finds radius and radial rate at end time for given coefficents.
    
    Args:
        a0, a1, a2, c1_radial, c2_radial: Coefficients which describe the commanded
          radial acceleration of the spacecraft over time.
        Tgo: [s] Time until engine cut-off. Equivalent to terminal time
          T subtracted by current time sample_t.
        r0: [m] Radius of spacecraft at time sample_t.
        r_dot_0: [m/s] Radial rate of spacecraft at time sample_t.
    
    Returns:
        r_T, r_dot_T: Expected radius and radial rate of spacecraft at
            end time T.
    """
    f11 = a0*Tgo + a1*Tgo**2/2 + a2*Tgo**3/3
    f21 = a0*Tgo**2/2 + a1*Tgo**3/3 + a2*Tgo**4/4
    f12 = f21
    f22 = a0*Tgo**3/3 + a1*Tgo**4/4 + a2*Tgo**5/5

    r_dot_T = r_dot_0 + f11*c1_radial + f12*c2_radial
    r_T = r0 + r_dot_0*Tgo + f21*c1_radial + f22*c2_radial

    return r_T, r_dot_T

class RadialYawGuidanceGroup(om.Group):
    def setup(self):
        self.add_subsystem('radial_yaw_guidance', RadialYawGuidance(), promotes=['*'])

    
class TestRadialYawGuidance(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(RadialYawGuidanceGroup())
        self.prob.setup()
        set_radial_yaw_guidance_default(self.prob)

    def test_case_1(self):
        """ Tests stationary start.
            
        Compares target state against calculated values for r_T
        and r_dot_T.

        """
        r0 = 1737.4e3
        self.prob['sample_x'] = np.array([r0, 0, 0])
        self.prob['sample_v'] = np.array([0, 0, 0])
        self.prob['target_r_T'] = r0 + 18.52e3
        self.prob['target_r_dot_T'] = 0
        self.prob['T'] = 438
        self.prob.run_model()
        a0, a1, a2, c1_radial, c2_radial = get_radial_guidance_coefficients(self.prob)


        # Check radial guidance coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = calculate_final_radial_state(
            a0, a1, a2, c1_radial, c2_radial, Tgo, r0, r_dot_0)
        
        r_T_residual = r_T_calculated - self.prob['target_r_T']
        r_dot_T_residual = r_dot_T_calculated - self.prob['target_r_dot_T']

        tol = 1e-8
        self.assertTrue(almost_equal(r_T_residual, 0, tol))
        self.assertTrue(almost_equal(r_dot_T_residual, 0, tol))

    def test_case_2(self):
        """ Tests RadialYawGuidance() mid-flight. 

        Compares target state against calculated values for r_T
        and r_dot_T.

        """
        # Test mid-flight, compare target state against manually calculated values 
        # for r_T and r_dot_T
        start_altitude = 9.0e3
        r0 = 1737.4e3
        r_t = r0 + start_altitude
        self.prob['sample_x'] = np.array([r_t, 0, 0])
        self.prob['sample_v'] = np.array([0, 0, 0])
        self.prob['target_r_T'] = r0 + 18.52e3
        self.prob['target_r_dot_T'] = 0
        self.prob['sample_t'] = 200
        self.prob['T'] = 438
        self.prob.run_model()
        a0, a1, a2, c1_radial, c2_radial = get_radial_guidance_coefficients(self.prob)


        # Check radial guidance coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = calculate_final_radial_state(
            a0, a1, a2, c1_radial, c2_radial, Tgo, r_t, r_dot_0)
        
        r_T_residual = r_T_calculated - self.prob['target_r_T']
        r_dot_T_residual = r_dot_T_calculated - self.prob['target_r_dot_T']

        tol = 1e-8
        self.assertTrue(almost_equal(r_T_residual, 0, tol))
        self.assertTrue(almost_equal(r_dot_T_residual, 0, tol))


if __name__ == '__main__':
    unittest.main()