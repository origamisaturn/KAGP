import unittest
import os.path

from gcherry.main_script import _run_cmd
from gcherry.guidance_components import InfeasibleError

class SubstituteArgumentParser():
    """ Class to emulate the object output by parser.parse_args() in 
    gcherry.main_script.gcherry_cmd(). Mainly for inputting config 
    files. """
    config_paths: list[str]
    nolog: bool
    plotlog: bool

    def __init__(self, config_paths: list[str]):
        self.config_paths = config_paths
        self.nolog = True
        self.plotlog = False

def checkBadInput(config_path):
    """ Returns true if expected InfeasibleError is thrown.
    
    Args:
        config_path: string, path to bad input file.

    """
    args = SubstituteArgumentParser([config_path])
    expectedException = False
    try:
        _run_cmd(args)
    except InfeasibleError:
        expectedException = True
    return expectedException

class TestGCherryCmd(unittest.TestCase):
    """ Tests _run_cmd() function. """
    def setUp(self):
        current_dir = os.path.dirname(__file__)
        self.input_dir = os.path.join(current_dir, 'input')
        self.bad_input_dir = os.path.join(current_dir, 'bad_input')

    def test_debug_ascent_1_nominal(self):
        """ Basic test to check run a good configuration without 
        throwing an exception. """
        args = SubstituteArgumentParser([
            os.path.join(self.input_dir, 
                'test_debug_ascent_1_scenario_1.yaml')
        ])
        _run_cmd(args)

    def test_orbit_targeting_ascent_nominal(self):
        """ Basic test to check run a good configuration without 
        throwing an exception. """
        args = SubstituteArgumentParser([
            os.path.join(self.input_dir, 
                'test_orbit_targeting_ascent_scenario_1.yaml')
        ])
        _run_cmd(args)

    def test_bad_input_debug_ascent_1(self):
        """ Test that checks for throwing of InfeasibleError in response
         to bad configs.  """
        config_paths = []
        for i in range(1, 6):
            config_paths.append(os.path.join(self.bad_input_dir, 
                'test_debug_ascent_1_bad_input_{}.yaml'.format(i)))
        for config_path in config_paths:
            exBool = checkBadInput(config_path)
            self.assertTrue(exBool)

    def test_bad_input_orbit_targeting_ascent(self):
        """ Test that checks for throwing of InfeasibleError in response
         to bad configs.  """        
        config_paths = []
        for i in range(1, 6):
            config_paths.append(os.path.join(self.bad_input_dir, 
                'test_orbit_targeting_ascent_bad_input_{}.yaml'.format(i)))
        for config_path in config_paths:
            exBool = checkBadInput(config_path)
            self.assertTrue(exBool)

if __name__ == '__main__':
    unittest.main()