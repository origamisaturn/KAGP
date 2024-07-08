import unittest
import numpy as np
import openmdao.api as om

from gcherry.guidance_components import (
    RadialYawGuidance,
    PitchHeadingQuery,
    TimeToGo,
    VThetaSolver,
    OuterLoopGroupRefactor,
    OrbitGuidanceGroup
)
from gcherry.transform import global2perifocal_rot
from gcherry.log_utils import almost_equal 


# See test_debug_ascent_1_scenario_1.yaml
def _set_radial_yaw_guidance_default(prob):
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
        'target_r_T': 1785.0e+3,
        'target_r_dot_T': 0,
        'target_lan': 0,
        'target_inc': 0,
        'v_e': 3893.24005,
        'm_dot': 0.420729258654369,
        'm0': 500,
        'T': 438}
    for key, value in input_dict.items():
        prob[key] = value

def _get_radial_guidance_coefficients(prob):
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

def _calculate_final_radial_state(a0, a1, a2, c1_radial, c2_radial, Tgo, r0, r_dot_0):
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
        _set_radial_yaw_guidance_default(self.prob)

    def test_case_1(self):
        """ Tests stationary start.
            
        Compares target state against calculated values for r_T
        and r_dot_T.

        """
        r0 = 1737.4e3
        self.prob['T'] = 438
        self.prob.run_model()
        a0, a1, a2, c1_radial, c2_radial = _get_radial_guidance_coefficients(self.prob)


        # Check radial guidance coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = _calculate_final_radial_state(
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
        a0, a1, a2, c1_radial, c2_radial = _get_radial_guidance_coefficients(self.prob)


        # Check radial guidance coefficents have expected final state.
        Tgo = self.prob['T'] - self.prob['sample_t']
        r_dot_0 = 0

        r_T_calculated, r_dot_T_calculated = _calculate_final_radial_state(
            a0, a1, a2, c1_radial, c2_radial, Tgo, r_t, r_dot_0)
        
        r_T_residual = r_T_calculated - self.prob['target_r_T']
        r_dot_T_residual = r_dot_T_calculated - self.prob['target_r_dot_T']

        tol = 1e-8
        self.assertTrue(almost_equal(r_T_residual, 0, tol))
        self.assertTrue(almost_equal(r_dot_T_residual, 0, tol))


# See test_debug_ascent_1_scenario_1.yaml
def _set_pitch_query_scenario_1(prob):
    a0, a1, a2 = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06
    )
    c1_radial, c2_radial = (
        -0.325367567512005,	
        0.00156271732299607
    )
    c1_yaw, c2_yaw = (0, 0)
    # Lunar radius
    r0 = 1737.4e3
    input_dict = {'query_x': [r0, 0, 0],
                  'query_v': [0, 0, 0],
                  'query_t': 0,
                  'target_lan': 0,
                  'target_inc': 0,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500,
                  'T': 438,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'target_r_T': 1785e+3,
                  'target_r_dot_T': 0}
    for key, value in input_dict.items():
        prob[key] = value

# See test_debug_ascent_1_scenario_2.yaml
def _set_pitch_query_scenario_2(prob):
    a0, a1, a2 = (
        5.4192248771367,
        -0.00754333097672137,
        1.04999964966261E-05
    )
    c1_radial, c2_radial = (
        -0.236081903268302,
        0.00109625350418313
    )
    c1_yaw, c2_yaw = (
        -0.144528539161392,
        0.000644824881978622
    )
    # Lunar radius
    r0 = 1737.4e3
    input_dict = {'query_x': [
                    1481773.78741417, 
                    -855502.4950417, 
                    301696.34387852],
                  'query_v': [0, 0, 0],
                  'query_t': 0,
                  'target_lan': 4.363323129985824,
                  'target_inc': 0.19198621771937624,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500,
                  'T': 470,
                  'a0': a0,
                  'a1': a1,
                  'a2': a2,
                  'c1_radial': c1_radial,
                  'c2_radial': c2_radial,
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'target_r_T': 1785e+3,
                  'target_r_dot_T': 20.0}
    for key, value in input_dict.items():
        prob[key] = value
    

class PitchHeadingQueryGroup(om.Group):
    def setup(self):
        self.add_subsystem('pitch_heading_query', PitchHeadingQuery(), promotes=['*'])


class TestPitchQuery(unittest.TestCase):
    def test_case_1(self):
        prob = om.Problem(PitchHeadingQueryGroup())
        prob.setup()
        _set_pitch_query_scenario_1(prob)
        tol = 1e-8

        pitch_expected_1 = 1.18447886794603
        heading_expected_1 = np.deg2rad(90)
        prob['query_t'] = 0
        prob.run_model()
        pitch_calc_1 = prob['cmd_pitch'][0]
        heading_calc_1 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.728365591911019
        heading_expected_2 = np.deg2rad(90)
        prob['query_t'] = 100
        prob['query_x'] = [
            1743371.45973407, 
            9064.77377033883, 
            5.5505786640278E-13
        ]
        prob['query_v'] = [
            108.553696158295, 
            204.538775905435, 
            1.25244257082427E-14
        ]
        prob.run_model()
        pitch_calc_2 = prob['cmd_pitch'][0]
        heading_calc_2 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))

    def test_case_2(self):
        prob = om.Problem(PitchHeadingQueryGroup())
        prob.setup()
        _set_pitch_query_scenario_2(prob)
        tol = 1e-8

        pitch_expected_1 = 1.02193090008912
        heading_expected_1 = 1.12721950511188
        prob['query_t'] = 0
        prob.run_model()
        pitch_calc_1 = prob['cmd_pitch'][0]
        heading_calc_1 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_1, pitch_expected_1, tol))
        self.assertTrue(almost_equal(heading_calc_1, heading_expected_1, tol))

        pitch_expected_2 = 0.689203268175016
        heading_expected_2 = 1.39664493990468
        prob['query_t'] = 100
        prob['query_x'] = [
            1490528.02411845,
            -849036.637175544,
            305753.440395322
        ]
        prob['query_v'] = [
            177.904433951903,
            149.952620578308,
            75.5017709619082
        ]
        prob.run_model()
        pitch_calc_2 = prob['cmd_pitch'][0]
        heading_calc_2 = prob['cmd_heading'][0]
        self.assertTrue(almost_equal(pitch_calc_2, pitch_expected_2, tol))
        self.assertTrue(almost_equal(heading_calc_2, heading_expected_2))

# See test_debug_ascent_1_scenario_1.yaml
def set_time_to_go_scenario1(prob):
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

# See test_debug_ascent_1_scenario_2.yaml
def set_time_to_go_scenario2(prob):
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

# Testing TimeToGo with the other components as it is
# intended to iteratively find the terminal time T.
class TimeToGoGroup(om.Group):
    def setup(self):
        self.add_subsystem('time_to_go', TimeToGo(), promotes=['*'])
        self.add_subsystem('radial_yaw_guidance', RadialYawGuidance(), promotes=['*'])
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])
        self.nonlinear_solver = om.NonlinearBlockGS()
        self.nonlinear_solver.options['maxiter'] = 100
        self.nonlinear_solver.options['atol'] = 1e-3

class TestTimeToGo(unittest.TestCase):
    # See set_time_to_go_scenario1()
    def test_case_1(self):
        # self.prob.model.time_to_go.is_first_entry = True
        T_expected = 438

        self.prob = om.Problem(TimeToGoGroup())
        self.prob.setup()
        set_time_to_go_scenario1(self.prob)

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

        self.prob = om.Problem(TimeToGoGroup())
        self.prob.setup()
        set_time_to_go_scenario2(self.prob)

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


# See test_debug_ascent_1_scenario_1.yaml
def set_v_theta_solver_scenario_1(prob):
    a0, a1, a2, c1_radial, c2_radial = (
        5.1881317827526,
        -0.00691370453645869,
        9.21320282887826E-06,
        -0.325367567512005,
        0.00156271732299607)
    c1_yaw, c2_yaw = (0, 0)
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
                  'c1_yaw': c1_yaw,
                  'c2_yaw': c2_yaw,
                  'mu': 4.9028e12,
                  'v_e': 3893.24005,
                  'm_dot': 0.420729258654369,
                  'm0': 500}
    for key, value in input_dict.items():
        prob[key] = value

# See test_debug_ascent_1_scenario_2.yaml
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
        self.add_subsystem('v_theta_solver', VThetaSolver(), promotes=['*'])

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
        tol = 1e-1
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
        tol = 2e-3
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
        tol = 1e-1
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
        tol = 2e-3
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
        tol = 2e-7
        self.assertTrue(almost_equal(v_theta_residual, 0, tol))


# See test_debug_ascent_1_scenario_1.yaml  
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

# See test_debug_ascent_1_scenario_2.yaml
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


# See test_orbit_targeting_ascent_scenario_1.yaml
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

# See test_orbit_targeting_ascent_scenario_2.yaml
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