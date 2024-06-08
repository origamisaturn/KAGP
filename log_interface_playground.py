import unittest
import pickle as pkl
from gcherry.log_interface import LogInterfaceRefactor
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("test_int_inter.pkl", "rb") as fh:
        log_interface = pkl.load(fh)
    df_log = log_interface.guidance_interface.problem.dataframe_log()
    derived_values = pd.DataFrame(log_interface.get_derived_values())
    tp_derived = derived_values['thrust_pitch']
    tp_cmd = df_log['outputs']['pitch_heading_query.cmd_pitch']
    df_err = log_interface.dataframe_error()
    log_interface.save_csv("test")
    log_interface.plot_error()
    log_interface.plot_derived()
    log_interface.plot_inputs()
    log_interface.plot_outputs()
    plt.show()
    print("here")