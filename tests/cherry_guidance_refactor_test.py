import unittest
import numpy as np
import math
import openmdao.api as om

import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, '..', 'core')))
from cherry_guidance_refactor import RadialControl


def set_radial_control_default(prob):
    """ Sets default input values for RadialControl.
    
    Args:
        openmdao.api.Problem containing only a RadialControl() explicit
            component.
    
    """
    prob['x'] = x0
    prob['v'] = v0
    prob['sample_t'] = 0
    # Other inputs
    prob['T'] = T_go_guess
    # Boundary conditions
    #   (Loosely following Apollo 11 LM ascent profile:
    #   https://history.nasa.gov/alsj/nasa-tnd-6846pt.1.pdf)
    prob['r_dot_T'] = 0
    prob['r_T'] = r0 + 18.52e3 # m
    # Physical constants 
    prob['mu'] = mu
    prob['v_e'] = v_e
    prob['m_dot'] = m_dot
    prob['m0'] = m0
    # Query Input
    prob['t'] = 10
    

class TestRadialControl(unittest.TestCase):
    def setUp(self):
        self.prob = om.Problem(RadialControl())
        self.prob.setup()
        set_radial_control_default(self.prob)

    def test_case_1():
        ...
    def test_case_2():
        ...