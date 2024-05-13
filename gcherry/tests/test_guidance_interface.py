import yaml
import numpy as np
import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface


if __name__ == '__main__':
    config_files = ["gcherry/tests/test_guidance_interface.yaml"]
    config = cfg.load_config(config_files)

    x0 = np.array([1737.4E+3, 0.0, 0.0])
    v0 = np.array([0.0, 0.0, 0.0])
    m0 = config.spacecraft.wet_mass

    guidance_interface = GCherryGuidanceInterface(config)
    thrust_mag, thrust_pitch, thrust_heading = (
        guidance_interface.get_command(0, np.concatenate((x0, v0, [m0]))))
    
    print("thrust_mat: {}".format(thrust_mag))
    print("thrust_pitch: {}".format(np.rad2deg(thrust_pitch)))
    print("thrust_heading: {}".format(np.rad2deg(thrust_heading)))
    