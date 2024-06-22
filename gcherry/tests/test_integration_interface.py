import unittest
import numpy as np
import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.integration_interface import IntegrationInterface
from gcherry.log_interface import LogInterfaceRefactor
from gcherry.log_utils_refactor import almost_equal

class TestIntegrationInterface(unittest.TestCase):
    def test_case_2(self):
        filenames = ["gcherry/tests/input/orbit_targeting_test_scenario2.yaml"]
        config = cfg.load_config(filenames)
        log_interface = LogInterfaceRefactor(config)
        guidance_interface = GCherryGuidanceInterface(config, log_interface)
        integration_interface = IntegrationInterface(config, guidance_interface, log_interface)

        integration_interface.run()

        # TODO: revisit structure of the log_interface object.
        # TODO: Choose between dataframe outputs and dict outputs.
        # Thinking dicts since they are more flexible.
        log_interface.save("test_integration_interface_test1.pkl")
        log_interface.save_csv("test_integration_interface_test1")

        df_deriv = log_interface.get_derived_values()
        # df_err = log_interface.dataframe_error()
        # df_prob = log_interface.guidance_interface.problem.dataframe_log()

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
    # filenames = ["gcherry/tests/input/orbit_targeting_test_scenario2.yaml"]
    # config = cfg.load_config(filenames)
    # log_interface = LogInterfaceRefactor(config)
    # guidance_interface = GCherryGuidanceInterface(config, log_interface)
    # integration_interface = IntegrationInterface(config, guidance_interface, log_interface)
    # t_res, y_res = integration_interface.run()
    # log_interface.save("test_500.pkl")
    unittest.main()