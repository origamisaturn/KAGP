import unittest
import numpy as np

import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.log_interface import LogInterfaceRefactor


class TestGuidanceInterface(unittest.TestCase):
    # Very basic test to make sure heading output makes sense.
    def test_funct__guidance_interface__heading(self):
        config_files = ["gcherry/tests/input/test_guidance_interface.yaml"]
        config = cfg.load_config(config_files)

        x0 = np.array([1737.4E+3, 0.0, 0.0E+3])
        v0 = np.array([0.0, 0.0, 0.0])
        m0 = config.spacecraft.wet_mass

        log_interface = LogInterfaceRefactor(config)
        guidance_interface = GCherryGuidanceInterface(config, log_interface)
        thrust_mag, thrust_pitch, thrust_heading = (
        guidance_interface.get_command(0, np.concatenate((x0, v0, [m0]))))
        self.assertAlmostEqual(thrust_heading, np.deg2rad(90))

        config.mission.inclination = np.deg2rad(90)
        guidance_interface = GCherryGuidanceInterface(config, log_interface)
        thrust_mag, thrust_pitch, thrust_heading = (
        guidance_interface.get_command(0, np.concatenate((x0, v0, [m0]))))
        self.assertAlmostEqual(thrust_heading, np.deg2rad(0))        


if __name__ == '__main__':
    unittest.main()
    # config_files = ["gcherry/tests/input/test_guidance_interface.yaml"]
    # config = cfg.load_config(config_files)

    # x0 = np.array([1737.4E+3, 0.0, 0.0E+3])
    # v0 = np.array([0.0, 0.0, 0.0])
    # m0 = config.spacecraft.wet_mass
    # initial_state = np.concatenate((x0, v0, [m0]))

    # log_interface = LogInterfaceRefactor(config)
    # guidance_interface = GCherryGuidanceInterface(config, log_interface)
    # guidance_interface.get_command(0, initial_state, outer_loop=True)

    # tspan = [0, 438]

    # guidance_func = lambda t, state: guidance_interface.get_command(t, state, outer_loop=False)
    # ode_func = lambda t, state: rocket_ode(t, state,
    #                                        config.body.gravitational_parameter,
    #                                        config.spacecraft.specific_impulse,
    #                                        config.spacecraft.thrust,
    #                                        guidance_func)
    # t_res, y_res = rk4(ode_func, tspan, initial_state, 1.0)
    # print(t_res[-1])
    # print(y_res[:, -1])
    # print(np.linalg.norm(y_res[:3, -1]))
    # log_interface.save("test.pkl")
