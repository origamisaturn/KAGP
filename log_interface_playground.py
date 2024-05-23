import unittest
import pickle as pkl
from gcherry.log_interface import LogInterfaceRefactor
import pandas as pd
import numpy as np

if __name__ == '__main__':
    with open("test_int_inter.pkl", "rb") as fh:
        log_interface = pkl.load(fh)
    df_log = log_interface.guidance_interface.problem.dataframe_log()
    derived_values = pd.DataFrame(log_interface.get_derived_values())
    tp_derived = derived_values['thrust_pitch']
    tp_cmd = df_log['outputs']['pitch_heading_query.cmd_pitch']
    df_err = log_interface.dataframe_error()
    print("here")