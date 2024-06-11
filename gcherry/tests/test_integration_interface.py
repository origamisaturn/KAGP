import unittest
import numpy as np
import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.integration_interface import IntegrationInterface
from gcherry.log_interface import LogInterfaceRefactor

if __name__ == '__main__':
    filenames = ["gcherry/tests/input/v_theta_solver_test_scenario2.yaml"]
    config = cfg.load_config(filenames)
    log_interface = LogInterfaceRefactor(config)
    guidance_interface = GCherryGuidanceInterface(config, log_interface)
    integration_interface = IntegrationInterface(config, guidance_interface, log_interface)
    t_res, y_res = integration_interface.run()
    log_interface.save("test_500.pkl")