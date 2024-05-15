import unittest
import numpy as np
import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.integration_interface import IntegrationInterface

if __name__ == '__main__':
    filenames = ["gcherry/tests/test_integration_interface.yaml"]
    config = cfg.load_config(filenames)
    guidance_interface = GCherryGuidanceInterface(config)
    integration_interface = IntegrationInterface(config, guidance_interface)
    integration_interface.run()