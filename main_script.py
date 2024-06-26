#!python
# Command line function
# TODO: Does the use of RK4 to estimate v_theta_T instead of a Taylor
# series make this guidance program no longer explicit?

import argparse

import gcherry.config as cfg
from gcherry.guidance_interface_refactor import GCherryGuidanceInterface
from gcherry.integration_interface import IntegrationInterface
from gcherry.log_interface import LogInterfaceRefactor
from gcherry.KSP_interface import KSPInterface


def gcherry_cmd():
    """ Main command for performing ascent guidance.
    """
    parser = argparse.ArgumentParser(prog='gcherry',
                                     description='PLACEHOLDER',
                                     )
    parser.add_argument('filenames', nargs='+')
    # TODO: add optional argument for setting custom log directory
    # parser.add_argument('--logpath')
    args = parser.parse_args()

    config = cfg.load_config(args.filenames)

    # TODO: Change the way LogInterfaceRefactor is initialized. Would
    # like for initialization to occur after logs are populated.
    # TODO: Consider changing the "Interface" name, may be confused with
    #  the datastructure type.
    log_interface = LogInterfaceRefactor(config)
    guidance_interface = GCherryGuidanceInterface(config, log_interface)
    if config.integrator:
        sim_interface = IntegrationInterface(config, guidance_interface, log_interface)
    elif config.ksp_interface:
        sim_interface = KSPInterface(config, guidance_interface, log_interface)
    else:
        raise(RuntimeError("No simulation defined in config."))
    sim_interface.run()
    # TODO: find cause of unit_vector() runtime warning.
    log_interface.save("test.pkl")
    log_interface.save_csv("test")

if __name__ == '__main__':
    gcherry_cmd()
