import yaml
import numpy as np
import unittest

import sys, os
sys.path.append(os.path.abspath('core'))

from gcherry.guidance_interface import generateGuidanceObj
from gcherry.krpc_client import KRPCClient
from gcherry.log import LogAnalyzer
import gcherry.config as cfg

filenames = ["gcherry/tests/input/ScriptKRPC.yaml",
             "gcherry/tests/input/newScriptKRPC2.yaml"]
config = cfg.load_config(filenames)
log_obj = LogAnalyzer(config)
guidance_obj = generateGuidanceObj(config)
sim_obj = KRPCClient(
    config, 
    guidance_obj, 
    log_obj)
sim_obj.run()