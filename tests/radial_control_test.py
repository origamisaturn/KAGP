import unittest
import numpy as np
import math
import openmdao.api as om

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', '..', 'core')))
from cherry_guidance_refactor import RadialControl


def almost_equal(val1, val2, tol=1e-8):
    arr_type = type(np.ndarray([]))
    if type(val1) == arr_type or type(val2) == arr_type:
        return (val1-val2 > -tol).all() and (val1-val2 < tol).all()
    else:
        return val1-val2 > -tol and val1-val2 < tol
    

def set_radial_control_default(prob):
    """ Sets default input values for RadialControl.
    
    Args:
        prob: openmdao.api.Problem containing only a RadialControl() explicit
            component.
    
    """
    r0 = 1737.4e3
    prob['x'] = np.array([r0, 0])
    prob['v'] = np.array([0, 0])
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = 438
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 0
    prob['r_T'] = r0 + 18.52e3 # m
    # Physical constants 
    prob['mu'] = 4.90e12
    prob['v_e'] = 3900
    prob['m_dot'] = 0.42
    prob['m0'] = 500

def get_radial_control_coefficients(prob):
    """ Convenience function for getting RadialControl() output.
    
    Args:
        prob: openmdao.api.Problem containing only a RadialControl() explicit
            component.
    
    Returns:
        (a0, a1, a2, c1, c2): Coefficients which describe the commanded
          radial acceleration of the spacecraft over time.
    
    """
    coefficient_list = []
    coefficient_keys = ['a0', 'a1', 'a2', 'c1', 'c2']
    for key in coefficient_keys:
        coefficient_list.append(prob[key])
    return tuple(coefficient_list)

def calculate_final_radial_state(a0, a1, a2, c1, c2, Tgo, r0, r_dot_0):
    """ Finds radius and radial rate at end time for given coefficents.
    
    Args:
        a0, a1, a2, c1, c2: Coefficients which describe the commanded
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

    r_dot_T = r_dot_0 + f11*c1 + f12*c2
    r_T = r0 + r_dot_0*Tgo + f21*c1 + f22*c2

    return r_T, r_dot_T

class RadialControlGroup(om.Group):
    def setup(self):
        self.add_subsystem('radial_control', RadialControl(), promotes=['*'])

    
class TestRadialControl(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(RadialControlGroup())
        self.prob.setup()
        set_radial_control_default(self.prob)

    def test_case_1(self):
        """ Tests stationary start.
            
        Compares target state against calculated values for r_T
        and r_dot_T.

        """
        r0 = 1737.4e3
        self.prob['x'] = np.array([r0, 0])
        self.prob['v'] = np.array([0, 0])
        self.prob['r_T'] = r0 + 18.52e3
        self.prob['r_dot_T'] = 0
        self.prob['T'] = 438
        self.prob.run_model()
        a0, a1, a2, c1, c2 = get_radial_control_coefficients(self.prob)


        # Check radial control coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = calculate_final_radial_state(
            a0, a1, a2, c1, c2, Tgo, r0, r_dot_0)
        
        r_T_residual = r_T_calculated - self.prob['r_T']
        r_dot_T_residual = r_dot_T_calculated - self.prob['r_dot_T']

        tol = 1e-8
        self.assertTrue(almost_equal(r_T_residual, 0, tol))
        self.assertTrue(almost_equal(r_dot_T_residual, 0, tol))

    def test_case_2(self):
        """ Tests RadialControl() mid-flight. 

        Compares target state against calculated values for r_T
        and r_dot_T.

        """
        # Test mid-flight, compare target state against manually calculated values 
        # for r_T and r_dot_T
        start_altitude = 9.0e3
        r0 = 1737.4e3
        r_t = r0 + start_altitude
        self.prob['x'] = np.array([r_t, 0])
        self.prob['v'] = np.array([0, 0])
        self.prob['r_T'] = r0 + 18.52e3
        self.prob['r_dot_T'] = 0
        self.prob['sample_t'] = 200
        self.prob['T'] = 438
        self.prob.run_model()
        a0, a1, a2, c1, c2 = get_radial_control_coefficients(self.prob)


        # Check radial control coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = calculate_final_radial_state(
            a0, a1, a2, c1, c2, Tgo, r_t, r_dot_0)
        
        r_T_residual = r_T_calculated - self.prob['r_T']
        r_dot_T_residual = r_dot_T_calculated - self.prob['r_dot_T']

        tol = 1e-8
        self.assertTrue(almost_equal(r_T_residual, 0, tol))
        self.assertTrue(almost_equal(r_dot_T_residual, 0, tol))


if __name__ == '__main__':
    unittest.main()