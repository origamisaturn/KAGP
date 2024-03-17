import sys, os
import pickle as pkl
import numpy as np
import pandas as pd
sys.path.append(os.path.abspath('core'))

from log_utils import *

mu = 4.9028e+12 # Moon

log_file = "ksp2d_031724_eng_prop_trf.pkl"
with open(log_file, 'rb') as fh:
    log = pkl.load(fh)

engine_data_file = "engine_est_out_031424"
with open(engine_data_file, 'rb') as fh:
    eng_dict = pkl.load(fh)

df_dict = log_to_dataframes(log, mu)
df_err = dataframe_errors(df_dict)
df_eng = pd.DataFrame(eng_dict)

def test_alpha(index):
    # a_x = df_dict['derived']['acc_x']
    # a_y = df_dict['derived']['acc_y']
    
    # pos = np.array([df_dict['state']['x'], df_dict['state']['y']])
    # a_g
    # a_g = get_gravity(df_dict, mu)
    # acc = get_acc(df_dict)
    # thrust_acc = acc - a_g
    # print(a_g[:, index])
    alpha = get_alpha(df_dict, mu)
    cmd_alpha = df_dict['outputs']['pitch_query.alpha']
    print(df_dict['derived']['t'][index])
    print(alpha[index])
    print(cmd_alpha[index])

t_20_3 = 100 #5000
test_alpha(t_20_3)
df_err.plot()
plot_dataframe_errors(df_dict)
df_dict['derived'].plot(x='t', y='thrust_acc')
df_eng.plot(x='t')

plt.show()

print("done")