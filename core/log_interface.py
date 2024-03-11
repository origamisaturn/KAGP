import pickle as pkl

class LogInterface:
    def __init__(self, input_data):
        self.log = {}
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
    
    def log_problem(self, openmdao_problem):
        inputs = self.log['inputs'].keys()
        outputs = self.log['outputs'].keys()
        for var_name in inputs:
            var_val = openmdao_problem[var_name][0]
            self.log['inputs'][var_name].append(var_val)
        for var_name in outputs:
            if type(openmdao_problem[var_name]) == type(dict()):
                for key in openmdao_problem[var_name]:
                    self.log['outputs'][var_name][key].append(var_val)
            else:
                var_val = openmdao_problem[var_name][0]
                self.log['outputs'][var_name].append(var_val)
        

    def log_state(self, state, t):
        x, y, vx, vy, m = tuple(state)
        state = {'t': t, 'x': x, 'y': y, 'vx': vx, 'vy': vy, 'm': m}
        for var_name in state:
            var_val = state[var_name]
            self.log['state'][var_name].append(var_val)
