import unittest
import numpy as np

import kagp.config as cfg
from kagp.guidance_interface import generate_guidance_obj


class TestGuidance(unittest.TestCase):
    # Very basic test to make sure heading output makes sense.
    def test_obj__OrbitTargetingAscent__heading_1(self):
        config_files = ["kagp/tests/input/test_orbit_targeting_ascent_scenario_1.yaml"]
        config = cfg.load_config(config_files)

        x0 = np.array([1737.4E+3, 0.0, 0.0E+3])
        v0 = np.array([0.0, 0.0, 0.0])
        m0 = config.spacecraft.wet_mass

        guidance_obj = generate_guidance_obj(config)
        _, _, thrust_heading = (
        guidance_obj.get_command(0, np.concatenate((x0, v0, [m0]))))
        self.assertAlmostEqual(thrust_heading, np.deg2rad(90))

        config.orbit_targeting_ascent.inclination = np.deg2rad(90)
        guidance_obj = generate_guidance_obj(config)
        _, _, thrust_heading = (
        guidance_obj.get_command(0, np.concatenate((x0, v0, [m0]))))
        self.assertAlmostEqual(thrust_heading, np.deg2rad(0))


if __name__ == '__main__':
    unittest.main()
