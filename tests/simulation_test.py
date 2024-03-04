import yaml
import numpy as np
import unittest

import sys, os
sys.path.append(os.path.abspath('core'))

from guidance_interface import TestGuidance2
from integration_interface import Integrator2DInterface

def relative_path(filepath):
    return os.path.abspath(os.path.join(__file__, '..', filepath))

input_filenames = [relative_path("simulation_test_input/ScriptKRPC.yaml"),
    relative_path("simulation_test_input/ScriptKRPCIntegrator.yaml")]

def load_input(filenames):
    input_data = {}
    # Add functions for checking yaml input and if filename exists
    for filename in filenames:
        with open(filename, 'r') as fh:
            yaml_input = yaml.safe_load(fh)
        if not isinstance(yaml_input, dict):
            raise BaseException
        input_data.update(yaml_input)

    return input_data


class TestSimulation(unittest.TestCase):
    def test_case_1(self):
        r0 = 1737.4e3
        mu = 4.90e12
        x0 = np.array([r0, 0])
        v0 = np.array([0, 0])
        m0 = 500
        t = 0

        state = np.concatenate((x0, v0, [m0]))

        input_data = load_input(input_filenames)
        test_guidance_interface = TestGuidance2(input_data)
        test_command = test_guidance_interface.get_command(state, t, logging=False)

        test_integration_interface = Integrator2DInterface(input_data, test_guidance_interface)
        test_integration_interface.run()

if __name__ == '__main__':
    unittest.main()