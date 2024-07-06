import unittest
import pickle as pkl
from gcherry.log import LogAnalyzer
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    filename = "test_integration_interface_test1"
    with open(filename + ".pkl", "rb") as fh:
        log_obj = pkl.load(fh)
    df_log = log_obj.guidance_log.problem.dataframe_log()
    derived_values = pd.DataFrame(log_obj.get_derived_values())
    tp_derived = derived_values['thrust_pitch']
    tp_cmd = df_log['outputs']['pitch_heading_query.cmd_pitch']
    df_err = log_obj.get_error_values()
    # log_obj.save_csv(filename)
    log_obj.plot_error()
    log_obj.plot_derived()
    log_obj.plot_inputs()
    log_obj.plot_outputs()
    plt.show()