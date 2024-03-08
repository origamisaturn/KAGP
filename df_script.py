import sys, os
import pickle as pkl
sys.path.append(os.path.abspath('core'))

from log_utils import *

log_file = "script_sim_test_logchange.pkl"
with open(log_file, 'rb') as fh:
    log = pkl.load(fh)

df_dict = log_to_dataframes(log)
df_err = dataframe_errors(df_dict)
df_err.plot()
plot_dataframe_errors(df_dict)

plt.show()

print("done")