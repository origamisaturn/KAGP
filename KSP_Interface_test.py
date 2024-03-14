import yaml
import numpy as np
import unittest

import sys, os
sys.path.append(os.path.abspath('core'))

from guidance_interface import TestGuidance3
from KSP_interface import KSP2DInterface
from log_interface import LogInterface

def relative_path(filepath):
    return os.path.abspath(os.path.join(__file__, '..', filepath))

input_filenames = [relative_path("spacecraft/ScriptKRPC.yaml"),
    relative_path("scenarios/ScriptKRPCIntegrator.yaml")]

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


input_data = load_input(input_filenames)
log_interface = LogInterface(input_data)
test_guidance_interface = TestGuidance3(input_data, log_interface)

test_integration_interface = KSP2DInterface(
    input_data, 
    test_guidance_interface, 
    log_interface)
test_integration_interface.run()