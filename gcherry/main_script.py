#!python
# Command line function
# TODO: Does the use of RK4 to estimate v_theta_T instead of a Taylor
# series make this guidance program no longer explicit?

import argparse
import os
import time
import pickle as pkl
import matplotlib.pyplot as plt

import kagp.config as cfg
from kagp.guidance_components import InfeasibleError
from kagp.guidance_interface import generate_guidance_obj, GuidanceBase
from kagp.sim_interface import generate_sim_obj
from kagp.log import LogAnalyzer


def kagp_cmd():
    """ Main command for performing ascent guidance. """
    parser = argparse.ArgumentParser(
        prog='kagp',
        description='A single-stage iterative ascent guidance program')
    subparsers = parser.add_subparsers()

    parser_run = subparsers.add_parser('run', help='runs ascent guidance')
    parser_run.add_argument('config_paths', nargs='+',
                            help='YAML configuration file path(s)')
    parser_run.add_argument('--nolog', action='store_true',
                            help='does not save log files after run')
    parser_run.add_argument('--plotlog', action='store_true',
                            help='runs plotlog subcommand after run')
    parser_run.set_defaults(func=_run_cmd)

    parser_plotlog = subparsers.add_parser('plotlog',
                                           help='plots log files')
    parser_plotlog.add_argument('log_dir',
                                help='path of directory containing log files')
    parser_plotlog.set_defaults(func=_plotlog_cmd)

    args = parser.parse_args()
    if getattr(args, 'func', None):
        # Prevents a failed convergence from generating a traceback.
        # Would prefer this be in _run_cmd, but test_cmd.py relies on
        # _run_cmd() passing on all exceptions
        try:
            args.func(args)
        except InfeasibleError as inst:
            print(inst)
    else:
        parser.print_help()


def _run_cmd(args):
    """ The 'run' subcommand. """
    config = cfg.load_config(args.config_paths)
    guidance_obj = generate_guidance_obj(config)
    sim_obj = generate_sim_obj(config, guidance_obj)
    sim_obj.run()
    log_obj = LogAnalyzer(config, guidance_obj.log, sim_obj.log)
    if not args.nolog:
        _save_log(config, guidance_obj, sim_obj, log_obj)
    if args.plotlog:
        _plot_log(log_obj)


def _plotlog_cmd(args):
    """ The 'plotlog' subcommand. """
    _, _, log_obj = _load_obj(args.log_dir)
    _plot_log(log_obj)


def _load_obj(save_dir):
    with open(os.path.join(save_dir, "config.pkl"), 'rb') as fh:
        config = pkl.load(fh)
    with open(os.path.join(save_dir, "guidance_obj_log.pkl"), 'rb') as fh:
        guidance_log_obj = pkl.load(fh)
    with open(os.path.join(save_dir, "sim_obj_log.pkl"), 'rb') as fh:
        sim_log_obj = pkl.load(fh)
    log_obj = LogAnalyzer(config, guidance_log_obj, sim_log_obj)
    return guidance_log_obj, sim_log_obj, log_obj


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
    log_obj.plot_derived()
    log_obj.plot_error()
    log_obj.plot_final_error(tspan=0.2)
    plt.show()


if __name__ == '__main__':
    kagp_cmd()
