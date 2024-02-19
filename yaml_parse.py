import yaml
import argparse
import numpy as np

import sys, os
sys.path.append(os.path.abspath('core'))

from guidance_interface import TestGuidance

parser = argparse.ArgumentParser(
        prog='ExampleProgram',
        description='Example description',
        epilog='text at the bottom of help')
parser.add_argument('filenames', 
        nargs = '+')

args = parser.parse_args()

input_data = {}
# Add functions for checking yaml input and if filename exists
for filename in args.filenames:
    with open(filename, 'r') as fh:
        yaml_input = yaml.safe_load(fh)
    if not isinstance(yaml_input, dict):
        raise BaseException
    input_data.update(yaml_input)

# Spacecraft goes to integrator, guidance
# mission goes to guidance
# simulator goes to integrator
print(input_data.keys())

r0 = 1737.4e3
mu = 4.90e12
x0 = np.array([r0, 0])
v0 = np.array([0, 0])
m0 = 500
T_go_guess = 438

state = np.concatenate((x0, v0, [m0]))
t = 0
T = 400

test_guidance_interface = TestGuidance(input_data)

test_guidance_interface._openmdao_problem['T'] = T
test_guidance_interface.get_command(state, t, logging=False)