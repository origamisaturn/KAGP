#!python
# Command line function
# TODO: Does the use of RK4 to estimate v_theta_T instead of a Taylor
# series make this guidance program no longer explicit?

import argparse

import gcherry.config as cfg
from gcherry.guidance_interface import generateGuidanceObj
from gcherry.integrator_sim import IntegratorSim
from gcherry.log import LogAnalyzer
from gcherry.krpc_client import KRPCClient


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
    log_obj = LogAnalyzer(config)
    guidance_obj = generateGuidanceObj(config)
    # TODO: create "generateSimObj"
    if config.integrator:
        sim_obj = IntegratorSim(config, guidance_obj)
    elif config.krpc_client:
        sim_obj = KRPCClient(config, guidance_obj)
    else:
        raise(RuntimeError("No simulation defined in config."))
    sim_obj.run()
    # TODO: find cause of unit_vector() runtime warning.
    log_obj.save("test.pkl")
    log_obj.save_csv("test")

if __name__ == '__main__':
    gcherry_cmd()
