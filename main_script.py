#!python
# Command line function
# TODO: Does the use of RK4 to estimate v_theta_T instead of a Taylor
# series make this guidance program no longer explicit?

import argparse
import os
import time
import pickle as pkl
import matplotlib.pyplot as plt

import gcherry.config as cfg
from gcherry.guidance_interface import generateGuidanceObj, GuidanceBase
from gcherry.integrator_sim import IntegratorSim
from gcherry.log import LogAnalyzer
from gcherry.krpc_client import KRPCClient


def gcherry_cmd():
    """ Main command for performing ascent guidance.
    """
    parser = argparse.ArgumentParser(prog='gcherry',
                                     description='PLACEHOLDER',
                                     )
    # TODO: add optional argument for setting custom log directory
    # parser.add_argument('--logpath')
    subparsers = parser.add_subparsers(help='sub-command help')

    parser_run = subparsers.add_parser('run', help='run help')
    parser_run.add_argument('config_paths', nargs='+')
    parser_run.add_argument('--log', action='store_true')
    parser_run.add_argument('--plotlog', action='store_true')
    parser_run.set_defaults(func=_run_cmd)
    
    parser_plotlog = subparsers.add_parser('plotlog', help='plotlog help')
    parser_plotlog.add_argument('log_dir')
    parser_plotlog.set_defaults(func=_plotlog_cmd)

    args = parser.parse_args()
    args.func(args)

def _run_cmd(args):
    config = cfg.load_config(args.config_paths)
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
    log_obj = LogAnalyzer(config, guidance_obj.log, sim_obj.log)
    if args.log:
        _save_log(config, guidance_obj, sim_obj, log_obj)
    if args.plotlog:
        _plot_log(log_obj)

def _plotlog_cmd(args):
    guidance_obj, sim_obj, log_obj = _load_obj(args.log_dir)
    _plot_log(log_obj)

def _load_obj(save_dir):
    config = pkl.load(os.path.join(save_dir, "config.pkl"))
    guidance_obj = pkl.load(os.path.join(save_dir, "guidance_obj_log.pkl"))
    sim_obj = pkl.load(os.path.join(save_dir, "sim_obj_log.pkl"))
    log_obj = LogAnalyzer(config, guidance_obj, sim_obj)
    return guidance_obj, sim_obj, log_obj

def _current_time_string():
    time_format = r"%m%d%y_%H%M%S"
    return time.strftime(time_format)

def _save_log(config: cfg.Config, guidance_obj: GuidanceBase, sim_obj, log_obj: LogAnalyzer):
    save_dir = os.path.join("logs", _current_time_string())
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    config.save_pkl(os.path.join(save_dir, "config.pkl"))
    guidance_obj.log.save_pkl(os.path.join(save_dir, "guidance_obj_log.pkl"))
    sim_obj.log.save_pkl(os.path.join(save_dir, "sim_obj_log.pkl"))
    log_obj.save_csv(save_dir)

def _plot_log(log_obj: LogAnalyzer):
    log_obj.plot_inputs()
    log_obj.plot_outputs()
    log_obj.plot_error()
    log_obj.plot_derived()
    plt.show()


# def report_cmd():
#     ...

# gcherry run config_file.yaml --savelog OPTIONALFOLDER
# gcherry logplot file_directory
# gcherry logplot guidance_obj.pkl sim_obj.pkl

if __name__ == '__main__':
    gcherry_cmd()
