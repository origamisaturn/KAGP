import pickle as pkl
from copy import deepcopy
from dataclasses import dataclass
import pandas as pd
import numpy as np
from gcherry.log_utils import get_time_steps, interpolate_state, get_derived_state

@dataclass
class StateLog:
    # position: list
    # velocity: list
    # mass: list
    state_vector: list
    time: list

    def __init__(self):
        # self.position = {}
        # self.velocity = {}
        # self.mass = {}
        self.time = {}
        self.state_vector

    def log_state(self, t, state):
        if len(state) != 7:
            ValueError("State has incorrect length.")
        # self.position.append(state[:3])
        # self.velocity.append(state[3:6])
        # self.mass.append(state[7])
        self.state_vector.append(state)
        self.time.append(t)

    def get_position(self):
        np_state_vector = np.array(self.state_vector)
        return np_state_vector[:, :3].T

    def get_velocity(self):
        np_state_vector = np.array(self.state_vector)
        return np_state_vector[:, 3:6].T
    
    def get_mass(self):
        np_state_vector = np.array(self.state_vector)
        return np_state_vector[:, 7].T

    def get_time(self):
        return np.array(self.time).T

    def _flatten_dict():
        ...

    def get_derived_values(self, t, state):
        ...

@dataclass
class IntegrationInterfaceLog:
    state: StateLog

@dataclass
class OpenMDAOProblemLog:
    """ 
    Attributes:
        inputs and outputs must be a dict of lists. The lists may 
        contain elements of arbitrary type, but each element must 
        have the same shape if it is an array. """
    inputs: dict
    outputs: dict

    def __init__(self):
        self.inputs = {}
        self.outputs = {}

    def init_problem(self, openmdao_problem):
        """ Sets log dicts based on structure of openmdao_problem.

        Each key of self.inputs and self.outputs is a list. This must
        be called once before calling self.log_problem().
        
        Args:
            openmdao_problem: an OpenMDAO problem.

        """
        model = openmdao_problem.model
        inputs = model.list_inputs()
        outputs = model.list_outputs()
        for var in inputs:
            var_name = var[0]
            self.inputs[var_name] = list()
        for var in outputs:
            var_name = var[0]
            self.outputs[var_name] = list()

    def log_problem(self, openmdao_problem):
        """ Logs problem variables.
        
        Args:
            openmdao_problem: an OpenMDAO problem.


        """
        
        input_names = self.inputs.keys()
        for var_name in input_names:
            var_val = deepcopy(openmdao_problem[var_name])
            self.inputs[var_name].append(var_val)

        output_names = self.outputs.keys()
        for var_name in output_names:
            var_val = deepcopy(openmdao_problem[var_name])
            self.outputs[var_name].append(var_val)

    def dataframe_log(self):
        """ Returns dataframe version of log data. 
        
        Dict entries are converted into list, and key names are 
        incorporated into the column names.
        
        Array entries are converted into lists for each index, index
        is incorporated into the column names.
        
        """
        df_log = {}
        flat_inputs, flat_outputs = self._flatten_log()
        df_log['inputs'] = pd.DataFrame(flat_inputs)
        df_log['outputs'] = pd.DataFrame(flat_outputs)
        return df_log

    def _flatten_log(self):
        """ Converts log to a flattened format suitable for dataframes. 

        Returns:
            tuple of flattened inputs and flattened outputs, 
            each one a dict.

        """
        flat_inputs = self._flatten_dict('', self.inputs)
        flat_outputs = self._flatten_dict('', self.outputs)
        return (flat_inputs, flat_outputs)
    
    def _flatten_dict(self, var_name, var_val):
        """ Flattens var_val into a depth-1 dictionary.
        
        Args:
            var_name: Key associated with var_val. 
                Set to '' if this is first entry into _flatten_dict.
            var_val: Value associated with var_name.

        Returns:
            dictionary with depth of 1.
        """
        new_table = {}
        if type(var_val) == type(dict()):
            for key, val in var_val.items():
                if var_name != '':
                    new_var_name = var_name + "." + key
                else:
                    new_var_name = key
                out_table = self._flatten_dict(new_var_name, val)
                new_table.update(out_table)
        elif type(var_val) == type(list()):
            if type(var_val[0]) == type(dict()):
                # TODO: implement check that all dicts have same keys
                new_var_val = self._lod2dol(var_val)
                new_table = self._flatten_dict(var_name, new_var_val)
            elif len(np.shape(var_val)) > 1:
                new_table = self._flatten_array_list(var_name, var_val)
            else:
                new_table = self._flatten_element_list(var_name, var_val)
        else:
            raise ValueError("Dict does not contain dict or list.")
        return new_table
    
    @staticmethod
    def _lod2dol(lod):
        """ Transforms list of dicts to a dict of lists. 
        
        Args:
            lod: list of dictionaries. dicts must all have the same keys.
            Contents of dicts are arbitrary.

        Returns:
            dictionary of lists.

        Example:
            [ {'a': 2, 'b': True}, {'a': 3, 'b':False} ] becomes
            {'a': [2, 3], 'b':[True, False] }

        """
        dol = {}
        # init dol
        for key, val in lod[0]:
            dol[key] = []
        for dict_element in lod:
            for key, val in dict_element:
                dol[key].append(val)
        return dol

    @staticmethod
    def _flatten_array_list(var_name, var_val):
        """ Flattens list of arrays into depth-1 dict.
         
        Args:
            var_name: key of list of arrays
            var_val: list of arrays

        Returns:
            dictionary of depth 1, with keys containing index information
            and with values that are lists of non-iterables.
        """
        new_table = {}
        var_len = np.shape(var_val)[1]
        np_var_val = np.array(var_val)
        for i in range(var_len):
            # only add index if there is more than one index per entry
            if var_len == 1:
                new_key = var_name
            else:
                new_key = "{}[{}]".format(var_name, i)
            new_val = list(np_var_val[:, i])
            new_table[new_key] = new_val
        return new_table
    
    @staticmethod
    def _flatten_element_list(var_name, var_val):
        """ Assigns list of primitives into dict.
         
        Args:
            var_name: key
            var_val: list of non-iterables
        
        Returns:
            dictionary with key var_name and value var_val.
        """
        new_table = {}
        new_table[var_name] = deepcopy(var_val)
        return new_table


@dataclass   
class GuidanceInterfaceLog:
    problem: OpenMDAOProblemLog
    def __init__(self):
        self.problem = OpenMDAOProblemLog()

    def init_problem(self, openmdao_problem):
        self.problem.init_problem(openmdao_problem)

    def log_problem(self, openmdao_problem):
        self.problem.log_problem(openmdao_problem)

@dataclass
class LogInterfaceRefactor:
    guidance_interface: GuidanceInterfaceLog
    integration_interface: IntegrationInterfaceLog
    def __init__(self, config):
        self.guidance_interface = GuidanceInterfaceLog()

    def save(self, save_path):
        with open(save_path, 'wb') as fh:
            pkl.dump(self, fh)

    def df_error(self):
        """ Dataframe of error between predicted and actual values. 
        
        Extracts data from guidance, and derives values based on stored
        simulation state, to make a table of error values.

        Returns:
        Dataframe with independent column t and several
        error columns.

        """
        ...


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
    
