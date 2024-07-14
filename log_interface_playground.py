import unittest
import pickle as pkl
from gcherry.log import LogAnalyzer
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from main_script import _load_obj

if __name__ == '__main__':
    dirpath = "logs/071424_013812"
    guidance_obj, sim_obj, log_obj = _load_obj(dirpath)

    log_obj.save_csv("testOut071324")
    # log_obj.plot_error()
    log_obj.plot_final_error()
    # log_obj.plot_derived()
    # log_obj.plot_shared_derived()
    # log_obj.plot_inputs()
    # log_obj.plot_outputs()

    print(log_obj.get_final_error_table())
    plt.show()