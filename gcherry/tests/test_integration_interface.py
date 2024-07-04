import unittest

import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.integration_interface import IntegrationInterface
from gcherry.log_interface import LogInterfaceRefactor
from gcherry.log_utils_refactor import almost_equal


class TestIntegrationInterface(unittest.TestCase):
    def test_case_1(self):
        filenames = ["gcherry/tests/input/test_debug_ascent_1_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_interface = GCherryGuidanceInterface(config)
        integration_interface = IntegrationInterface(config, guidance_interface)

        integration_interface.run()

        log_interface = LogInterfaceRefactor(config, 
                guidance_interface.log, integration_interface.log)
        log_interface.save("test_integration_interface_test1.pkl")
        log_interface.save_csv("test_integration_interface_test1")

        df_deriv = log_interface.get_derived_values()
        calc_r_T = df_deriv['radius'].iloc[-1]
        calc_r_dot_T = df_deriv['r_dot'].iloc[-1]
        calc_lan = df_deriv['lan'].iloc[-1]
        calc_inc = df_deriv['inc'].iloc[-1]
        self.assertTrue(almost_equal(calc_r_T, 1785.00e+3, 1e-6))
        self.assertTrue(almost_equal(calc_r_dot_T, 20.0, 1e-6))
        self.assertTrue(almost_equal(calc_lan, 4.363323129985824, 1e-6))
        self.assertTrue(almost_equal(calc_inc, 0.19198621771937624, 1e-6))

    def test_case_2(self):
        filenames = ["gcherry/tests/input/test_orbit_targeting_ascent_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_interface = GCherryGuidanceInterface(config)
        integration_interface = IntegrationInterface(config, guidance_interface)

        integration_interface.run()

        log_interface = LogInterfaceRefactor(config, 
                guidance_interface.log, integration_interface.log)
        log_interface.save("test_integration_interface_test2.pkl")
        log_interface.save_csv("test_integration_interface_test2")

        df_deriv = log_interface.get_derived_values()

        # TODO: add integration step option
        # TODO: Make more robust to change in terminal time T.
        tol = 1e-5
        calc_semi_major_axis = df_deriv['semi_major_axis'].iloc[-1]
        calc_ecc = df_deriv['ecc'].iloc[-1]
        calc_inc = df_deriv['inc'].iloc[-1]
        calc_lan = df_deriv['lan'].iloc[-1]
        calc_argp = df_deriv['argp'].iloc[-1]
        df_deriv = log_interface.get_derived_values()
        self.assertTrue(almost_equal(calc_semi_major_axis, 1785e+3, 1e-1))
        self.assertTrue(almost_equal(calc_ecc, 0.0028011204481792717, 1e-6))
        self.assertTrue(almost_equal(calc_inc, 0.19198621771937624, 1e-6))
        self.assertTrue(almost_equal(calc_lan, 4.363323129985824, 1e-6))
        self.assertTrue(almost_equal(calc_argp, 0.3, 1e-5))


if __name__ == '__main__':
    unittest.main()