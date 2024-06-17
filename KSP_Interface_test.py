import yaml
import numpy as np
import unittest

import sys, os
sys.path.append(os.path.abspath('core'))

from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.KSP_interface import KSPInterface
from gcherry.log_interface import LogInterfaceRefactor
import gcherry.config as cfg

filenames = ["gcherry/tests/input/ScriptKRPC.yaml",
             "gcherry/tests/input/newScriptKRPC.yaml"]
config = cfg.load_config(filenames)
log_interface = LogInterfaceRefactor(config)
guidance_interface = GCherryGuidanceInterface(config, log_interface)
integration_interface = KSPInterface(
    config, 
    guidance_interface, 
    log_interface)
integration_interface.run()