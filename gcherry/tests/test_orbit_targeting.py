import unittest
import numpy as np
import openmdao.api as om
from gcherry.cherry_guidance_refactor import (
    OrbitTargeting, 
    OuterLoopComponent)

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

class OrbitTargetingGroup(om.Group):
    def setup(self):
        self.add_subsystem('orbit_targeting', OrbitTargeting(), promotes=['*'])
        self.add_subsystem('outer_loop', OuterLoopComponent(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options['maxiter'] = 100
        self.nonlinear_solver.options['atol'] = 1e-3

class TestOrbitTargetingGroup(unittest.TestCase):
    def test_case_1(self):
        self.prob = om.Problem(OrbitTargetingGroup())
        self.prob.setup()
        set_orbit_targeting_scenario_1(self.prob)

        # Test from stationary start
        self.prob.run_model()
        print("here")

if __name__ == '__main__':
    unittest.main()