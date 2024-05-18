import unittest
import pickle as pkl
from gcherry.log_interface import LogInterfaceRefactor

if __name__ == '__main__':
    with open("test.pkl", "rb") as fh:
        log_interface = pkl.load(fh)
    df_log = log_interface.guidance_interface.problem.dataframe_log()
    print(df_log)
    print("here")