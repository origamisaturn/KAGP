import pickle as pkl
from copy import deepcopy
import pandas as pd
import numpy as np
from log_utils import get_time_steps, interpolate_state, get_derived_state

class LogInterface:
    def __init__(self, input_data=None):
        self.log = {}
        if input_data:
            self.log_save_path = input_data['simulator']['log_path']

    def save(self):
        with open(self.log_save_path, 'wb') as fh:
            pkl.dump(self.log, fh) 

    def init_guidance_log(self, openmdao_problem):
        self.log['inputs'] = {}
        self.log['outputs'] = {}
        model = openmdao_problem.model
        inputs = model.list_inputs()
        outputs = model.list_outputs()
        for var in inputs:
            var_name = var[0]
            var_val = var[1]['val']
            self.log['inputs'][var_name] = list()
        for var in outputs:
            var_name = var[0]
            var_val = var[1]['val']
            # For _debug output
            if type(var_val) == type(dict()):
                self.log['outputs'][var_name] = {}
                for key in var_val:
                    self.log['outputs'][var_name][key] = []
            else:
                self.log['outputs'][var_name] = list()

    def init_sim_log(self):
        self.log['state'] = {}
        state_names = ['t', 'x', 'y', 'vx', 'vy', 'm']
        for var_name in state_names:
            self.log['state'][var_name] = list()
        return self.log
    
    # modified this sunday, hopefully now compatible with dataframe
    def log_problem(self, openmdao_problem):
        inputs = self.log['inputs'].keys()
        outputs = self.log['outputs'].keys()

        for var_name in inputs:
            var_val = deepcopy(openmdao_problem[var_name])
            self.log['inputs'][var_name].append(var_val)

        for var_name in outputs:
            if type(openmdao_problem[var_name]) == type(dict()):
                for key, value in openmdao_problem[var_name].items():
                    self.log['outputs'][var_name][key].append(deepcopy(value))
            else:
                var_val = deepcopy(openmdao_problem[var_name])
                self.log['outputs'][var_name].append(var_val)
        

    def log_state(self, state, t):
        x, y, vx, vy, m = tuple(state)
        state = {'t': t, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'm': m}
        for var_name in state:
            var_val = state[var_name]
            self.log['state'][var_name].append(var_val)

    # dataframe stuff
            #FIX
    def dataframe_errors(df_dict):
        inp = df_dict['inputs']
        out = df_dict['outputs']
        der = df_dict['derived']
        t = df_dict['derived']['t']
        t_step = get_time_steps(df_dict['derived']['t'])
        r_dot_dot_error = (df_dict['derived']['r_dot_dot'] - 
                        df_dict['outputs']['pitch_query._debug.r_dot_dot'])
        return pd.DataFrame(
            {'t': t,
            't_step': t_step,
            'r_dot_dot_err': r_dot_dot_error}
        )

    def log_to_dataframes(self):
        dataframes = {}

        formatted_log = self.format_log()
        interpolate_times = formatted_log['inputs']['pitch_query.t']
        interp_state = interpolate_state(formatted_log, interpolate_times)
        formatted_log['state'] = interp_state
        derived_state = get_derived_state(formatted_log, mu)

        dataframes['outputs'] = pd.DataFrame(formatted_log['outputs'])
        dataframes['inputs'] = pd.DataFrame(formatted_log['inputs'])
        dataframes['state'] = pd.DataFrame(formatted_log['state'])
        dataframes['derived'] = pd.DataFrame(derived_state)

        return dataframes
    
    # Consider moving interpolation here, also adding error calc
    def format_log(self):
        """ Convert to format for dataframe 
        
        Makes everything into a 1-D list. """
        new_log = {}
        log = self.log

        inputs = log['inputs']
        outputs = log['outputs']
        state = log['state']

        new_log['inputs'] = self.format_prob_log(inputs)
        new_log['outputs'] = self.format_prob_log(outputs)
        new_log['state'] = deepcopy(state)

        return new_log
    
    def format_prob_log(self, prob_log):
        new_prob_log = {}
        for var_name, var_val in prob_log.items():
            if type(var_val) == type(dict()):
                for dict_key, dict_val in var_val.items():
                    new_key = var_name + '.' + dict_key
                    new_val = dict_val
                    new_prob_log[new_key] = new_val
            else:
                var_len = np.shape(var_val)[1]
                np_var_val = np.array(var_val)
                if var_len != 1:
                    for i in range(var_len):
                        new_key = "{}[{}]".format(var_name, i)
                        new_val = list(np_var_val[:, i])
                        new_prob_log[new_key] = new_val
                else:
                    new_val = list(np_var_val[:, 0])
                    new_prob_log[var_name] = new_val
        return new_prob_log
    
