import unittest

import gcherry.config as cfg
from gcherry.guidance_interface import generateGuidanceObj
from gcherry.sim_interface import generateSimObj
from gcherry.log import LogAnalyzer
from gcherry.log_utils import almost_equal


class TestIntegratorSim(unittest.TestCase):
    def test_case_1(self):
        filenames = ["gcherry/tests/input/test_debug_ascent_1_scenario_1.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generateGuidanceObj(config)
        sim_obj = generateSimObj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config, 
                guidance_obj.log, sim_obj.log)

        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_r_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_r_dot_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))

    def test_case_2(self):
        filenames = ["gcherry/tests/input/test_debug_ascent_1_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generateGuidanceObj(config)
        sim_obj = generateSimObj(config, guidance_obj)

        sim_obj.run()

        log_obj = LogAnalyzer(config, 
                guidance_obj.log, sim_obj.log)

        ferr = log_obj.get_final_error_table()
        self.assertTrue(almost_equal(ferr['target_r_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_r_dot_T_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_lan_err'], 0.0, 1e-6))
        self.assertTrue(almost_equal(ferr['target_inc_err'], 0.0, 1e-6))

    def test_case_3(self):
        filenames = ["gcherry/tests/input/test_orbit_targeting_ascent_scenario_1.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generateGuidanceObj(config)
        sim_obj = generateSimObj(config, guidance_obj)

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
        filenames = ["gcherry/tests/input/test_orbit_targeting_ascent_scenario_2.yaml"]
        config = cfg.load_config(filenames)
        guidance_obj = generateGuidanceObj(config)
        sim_obj = generateSimObj(config, guidance_obj)

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


if __name__ == '__main__':
    unittest.main()