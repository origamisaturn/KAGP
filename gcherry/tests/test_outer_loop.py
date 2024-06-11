from gcherry.cherry_guidance_refactor import (
    VThetaSolver,
    TimeToGo,
    RadialYawGuidance,
    PitchHeadingQuery,
    OuterLoopComponent)

# Takeoff from lunar surface along equator to a position with 0 r_dot.
# Only radial guidance, no yaw.    
def set_outer_loop_component_scenario1(prob):
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
def set_outer_loop_component_scenario2(prob):
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
    def test_case_1(self):
        ...