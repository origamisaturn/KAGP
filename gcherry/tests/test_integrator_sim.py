import unittest
import numpy as np

import kagp.config as cfg
from kagp.guidance_interface import generate_guidance_obj
from kagp.sim_interface import generate_sim_obj
from kagp.log import LogAnalyzer
from kagp.log_utils import almost_equal


class TestIntegratorSim(unittest.TestCase):
    def test_case_1(self):
        filenames = ["kagp/tests/input/test_debug_ascent_1_scenario_1.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generate_guidance_obj(config)
        sim_obj = generate_sim_obj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config,
                guidance_obj.log, sim_obj.log)

        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_r_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_r_dot_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))

    def test_case_2(self):
        filenames = ["kagp/tests/input/test_debug_ascent_1_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generate_guidance_obj(config)
        sim_obj = generate_sim_obj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config,
                guidance_obj.log, sim_obj.log)

        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_r_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_r_dot_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))

    def test_case_3(self):
        filenames = ["kagp/tests/input/test_orbit_targeting_ascent_scenario_1.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generate_guidance_obj(config)
        sim_obj = generate_sim_obj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config,
                guidance_obj.log, sim_obj.log)

        # TODO: add integration step option
        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_pe_err'], 0.0, 1e-5))
        self.assertTrue(almost_equal(ferr['target_ap_err'], 0.0, 1e-2))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))
        # argp is degenerate when inc is 0.0

    def test_case_4(self):
        filenames = ["kagp/tests/input/test_orbit_targeting_ascent_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generate_guidance_obj(config)
        sim_obj = generate_sim_obj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config,
                guidance_obj.log, sim_obj.log)

        # TODO: add integration step option
        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_pe_err'], 0.0, 1e-2))
        self.assertTrue(almost_equal(ferr['target_ap_err'], 0.0, 1e-2))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_argp_err'], 0.0, 1e-5))

    def test_case_5(self):
        """ Apollo LM Ascent Stage sample trajectory. """
        filenames = ["kagp/tests/input/test_apollo_ascent_example.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generate_guidance_obj(config)
        sim_obj = generate_sim_obj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config,
                guidance_obj.log, sim_obj.log)

        ferr = log_obj.get_final_error_table()
        shared_derived = log_obj.get_shared_derived_values()
        derived = log_obj.get_derived_values()
        final_t = ferr['t']

        # Ascent trajectory burn time expected to be 7 minutes 18 seconds,
        # burnout altitude 60,000ft, true anomaly at 18 degrees
        self.assertTrue(almost_equal(final_t, 438, 17.0))
        self.assertTrue(almost_equal(
            np.interp(final_t, derived['t'], derived['radius']),
            1737.4e3 + 18.288e3,
            0.1e3 ))
        self.assertTrue(almost_equal(
            np.interp(final_t, shared_derived['t'], shared_derived['projected_nu']),
            np.deg2rad(18),
            np.deg2rad(0.2)
        ))

        self.assertTrue(almost_equal(ferr['target_pe_err'], 0.0, 1e-2))
        self.assertTrue(almost_equal(ferr['target_ap_err'], 0.0, 1e-2))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_argp_err'], 0.0, 1e-5))


if __name__ == '__main__':
    unittest.main()
