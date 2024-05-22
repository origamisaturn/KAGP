import unittest
import pickle as pkl
from gcherry.log_interface import LogInterfaceRefactor

if __name__ == '__main__':
    with open("test_int_inter.pkl", "rb") as fh:
        log_interface = pkl.load(fh)
    df_log = log_interface.guidance_interface.problem.dataframe_log()
    derived_values = log_interface.get_derived_values()
    df_err = log_interface.dataframe_error()
    print("here")