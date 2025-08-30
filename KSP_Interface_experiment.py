import yaml
import numpy as np
import unittest

import sys, os
sys.path.append(os.path.abspath('core'))

from kagp.guidance_interface import generateGuidanceObj
from kagp.krpc_client import KRPCClient
from kagp.log import LogAnalyzer
import kagp.config as cfg

filenames = ["kagp/tests/input/ScriptKRPC.yaml",
             "kagp/tests/input/newScriptKRPC2.yaml"]
config = cfg.load_config(filenames)
log_obj = LogAnalyzer(config)
guidance_obj = generateGuidanceObj(config)
sim_obj = KRPCClient(
    config, 
    guidance_obj, 
    log_obj)
sim_obj.run()