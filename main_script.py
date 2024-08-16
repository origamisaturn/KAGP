import argparse
import os
import time
import pickle as pkl
import matplotlib.pyplot as plt

import gcherry.config as cfg
from gcherry.guidance_interface import generateGuidanceObj, GuidanceBase
from gcherry.sim_interface import generateSimObj
from gcherry.log import LogAnalyzer


def gcherry_cmd():
    """ Main command for performing ascent guidance.
    """
    parser = argparse.ArgumentParser(prog='gcherry',
                                     description='A single-stage iterative ascent guidance program')
    subparsers = parser.add_subparsers()

    parser_run = subparsers.add_parser('run', help='runs ascent guidance')
    parser_run.add_argument('config_paths', nargs='+', help='YAML configuration file path(s)')
    parser_run.add_argument('--nolog', action='store_true', help='does not save log files after run')
    parser_run.add_argument('--plotlog', action='store_true', help='runs plotlog subcommand after run')
    parser_run.set_defaults(func=_run_cmd)
    
    parser_plotlog = subparsers.add_parser('plotlog', help='plots log files')
    parser_plotlog.add_argument('log_dir', help='path of directory containing log files')
    parser_plotlog.set_defaults(func=_plotlog_cmd)

    args = parser.parse_args()
    if getattr(args, 'func', None):
        args.func(args)
    else:
        parser.print_help()

def _run_cmd(args):
    config = cfg.load_config(args.config_paths)
    guidance_obj = generateGuidanceObj(config)
    sim_obj = generateSimObj(config, guidance_obj)
    sim_obj.run()
    # TODO: find cause of unit_vector() runtime warning.
    log_obj = LogAnalyzer(config, guidance_obj.log, sim_obj.log)
    if not args.nolog:
        _save_log(config, guidance_obj, sim_obj, log_obj)
    if args.plotlog:
        _plot_log(log_obj)

def _plotlog_cmd(args):
    guidance_obj, sim_obj, log_obj = _load_obj(args.log_dir)
    _plot_log(log_obj)

def _load_obj(save_dir):
    with open(os.path.join(save_dir, "config.pkl"), 'rb') as fh:
        config = pkl.load(fh)
    with open(os.path.join(save_dir, "guidance_obj_log.pkl"), 'rb') as fh:
        guidance_obj = pkl.load(fh)
    with open(os.path.join(save_dir, "sim_obj_log.pkl"), 'rb') as fh:
        sim_obj = pkl.load(fh)
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

if __name__ == '__main__':
    gcherry_cmd()
