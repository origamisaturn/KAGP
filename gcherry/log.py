import os
import typing
import pickle as pkl
from copy import deepcopy

import matplotlib
import pandas as pd
import numpy as np

from gcherry.config import Config
import gcherry.log_utils as log_utils
from gcherry.log_utils import plot_vars, interp_table


# TODO: Write documentation for this
class StateLog:
    """ Log for recording rocket state in simulations.
    
    Attributes:
        state_vectors: [m, m, m, m/s, m/s, m/s, kg] List of 7-element 
            lists. The seven elements are [x, y, z, vx, vy, vz, m],
            measured in the global frame.
        times: List of the time [s] corresponding to each state vector.

    """
    state_vectors: list
    times: list

    def __init__(self):
        self.times = []
        self.state_vectors = []

    def log_state(self, t, state):
        """ Logs a single time sample.
        
        Args:
            t: [s] Time, float.
            state: [m, m, m, m/s, m/s, m/s, kg] 7-element list of state
                at time t.
        """
        if len(state) != 7:
            raise ValueError("State has incorrect length.")
        self.state_vectors.append(state)
        self.times.append(t)

    def get_position(self):
        """ Gets array of positions.
        
        Returns:
            3xN array of positions, where each row is a position
            sample [x, y, z]. 
        """
        np_state_vectors = np.array(self.state_vectors)
        return np_state_vectors[:, :3].T

    def get_velocity(self):
        """ Gets array of velocities.
        
        Returns:
            3xN array of velocities, where each row is a velocity
            sample [vx, vy, vz]. 
        """
        np_state_vectors = np.array(self.state_vectors)
        return np_state_vectors[:, 3:6].T

    def get_mass(self):
        """ Gets array of masses.
        
        Returns:
            Length N array of mass.     
        """
        np_state_vectors = np.array(self.state_vectors)
        return np_state_vectors[:, 6].T

    def get_time(self):
        """ Gets array of times.
        
        Returns:
            Length N array of times.
        """
        return np.array(self.times).T

    def dataframe_log(self):
        """ Gets dataframe representation of StateLog.

        Returns:
            Dataframe containing columns for time, position, velocity,
            and mass.
        """
        t = self.get_time()
        pos = self.get_position()
        vel = self.get_velocity()
        m = self.get_mass()
        df_dict = {
            't': t,
            'x': pos[0, :],
            'y': pos[1, :],
            'z': pos[2, :],
            'vx': vel[0, :],
            'vy': vel[1, :],
            'vz': vel[2, :],
            'm': m
        }
        return pd.DataFrame(df_dict)

    def plot_state(self):
        """ Plots all the state log data. """
        t = self.get_time()
        pos = self.get_position()
        vel = self.get_velocity()
        m = self.get_mass()
        plot_dict = {
            'x': pos[0, :],
            'y': pos[1, :],
            'z': pos[2, :],
            'vx': vel[0, :],
            'vy': vel[1, :],
            'vz': vel[2, :],
            'm': m
        }
        plot_vars(plot_dict, t, columns=4)

class AutoPilotLog:
    pitch_errors: list
    heading_errors: list
    roll_errors: list
    errors: list
    target_pitches: list
    target_headings: list
    target_rolls: list
    # SpaceCenter TransformRotation
    # rpy -> quaternion -> TransformRotation(from autopilot to global 
    #   inertial) -> rpy
    # the errors should be the same, but maybe calculate it manual

    def __init__(self):
        self.pitch_errors = list()
        self.heading_errors = list()
        self.roll_errors = list()
        self.errors = list()
        self.target_pitches = list()
        self.target_headings = list()
        self.target_rolls = list()

    def log_autopilot(self, autopilot):
        self.pitch_errors.append(autopilot.pitch_error)
        self.heading_errors.append(autopilot.heading_error)
        self.roll_errors.append(autopilot.roll_error)
        self.errors.append(autopilot.error)
        self.target_pitches.append(autopilot.target_pitch)
        self.target_headings.append(autopilot.target_heading)
        self.target_rolls.append(autopilot.target_roll)

    def dataframe_log(self):
        df_dict = {
            'pitch_error': self.pitch_errors,
            'heading_error': self.heading_errors,
            'roll_error': self.roll_errors,
            'error': self.errors,
            'target_pitch': self.target_pitches,
            'target_heading': self.target_headings,
            'target_roll': self.target_rolls
        }
        return pd.DataFrame(df_dict)

class VesselLog:
    """I want:
    - moments of inertia
    - target rpy, actual rpy
    - target inertial rpy, actual inertial rpy
    - effected torque
    """
    # note the 'bounding_box' item, can be used for rendezvous pathfinding
    # 'rotation' gives the quaternion rotation in a certain reference frame
    # 'direction' unit vector of vehicle pointing in reference frame
    # 'angular_velocity' in given reference frame
    # the 'control' object can be use to directly affect the vehicle controls
    #   this might be affected by the same kind of delay that made "mecoshift"
    #   necessary. can also engage, disengage rcs
    # the 'autopilot' can give the target direction and target rpy error.
    #   have to set reference frame variable though.
    autopilot: AutoPilotLog
    times: list
    inertia_tensors: list
    rotations_surface: list
    rotations_global: list
    angular_velocities_body: list #worried about doing it in surface, if
    # it will be relative to inertial in surface axes

    def __init__(self):
        self.autopilot = AutoPilotLog()
        self.times = []
        self.inertia_tensors = []
        self.rotations_surface = []
        self.rotations_global = []
        self.angular_velocities_body = []

    # NOTE: inconsistent with simulationlog not having a log
    def log_vessel(self, t, vessel):
        self.autopilot.log_autopilot(vessel.auto_pilot)
        self.times.append(t)
        self.inertia_tensors.append(vessel.inertia_tensor)
        self.rotations_surface.append(list(
            vessel.rotation(vessel.surface_reference_frame)
        ))
        self.rotations_global.append(list(
            vessel.rotation(vessel.orbit.body.non_rotating_reference_frame)
        ))
        self.angular_velocities_body.append(list(
            vessel.angular_velocity(vessel.reference_frame)
        ))

    def dataframe_log(self):
        np_rot_global = np.array(self.rotations_global)
        np_rot_surf = np.array(self.rotations_surface)
        np_inert = np.array(self.inertia_tensors)
        np_w = np.array(self.angular_velocities_body)
        df_dict = {
            't': self.times,
            'q0': np_rot_global[:, 3],
            'q1': np_rot_global[:, 0],
            'q2': np_rot_global[:, 1],
            'q3': np_rot_global[:, 2],
            'q0_surf': np_rot_surf[:, 3],
            'q1_surf': np_rot_surf[:, 0],
            'q2_surf': np_rot_surf[:, 1],
            'q3_surf': np_rot_surf[:, 2],
            'I11': np_inert[:, 0],
            'I12': np_inert[:, 1],
            'I13': np_inert[:, 2],
            'I21': np_inert[:, 3],
            'I22': np_inert[:, 4],
            'I23': np_inert[:, 5],
            'I31': np_inert[:, 6],
            'I32': np_inert[:, 7],
            'I33': np_inert[:, 8],
            'w_x': np_w[:, 0],
            'w_y': np_w[:, 1],
            'w_z': np_w[:, 2]
        }
        df_vessel = pd.DataFrame(df_dict)
        df_autopilot = self.autopilot.dataframe_log()
        df_combined = pd.concat((df_vessel, df_autopilot), axis=1)
        return df_combined


class SimulationLog:
    """ Log used by SimulatorBase subclasses.
    
    Attributes:
        state: A StateLog object for logging vehicle state over time.
    """
    state: StateLog
    vessel: VesselLog

    def __init__(self):
        self.state = StateLog()
        self.vessel = VesselLog()

    def save_pkl(self, save_path):
        """ Convenience function for saving object as pickle file.
        
        Args:
            save_path: path-like object, save location for pickle file.
        """
        with open(save_path, 'wb') as fh:
            pkl.dump(self, fh)


class OpenMDAOProblemLog:
    """ Generic logging class for OpenMDAO problems.

    Attributes:
        inputs: dictionary. Keys are names of problem inputs, value of
            each key is a list of logged problem values.
        outputs: dictionary. Keys are names of problem outputs, value of
            each key is a list of logged problem values.
    """
    inputs: dict
    outputs: dict

    def __init__(self):
        """ Basic initialization. init_problem() must be called to setup
        this object.
        """
        self.inputs = {}
        self.outputs = {}

    def init_problem(self, openmdao_problem):
        """ Sets up attributes based on structure of openmdao_problem.

        Each key of self.inputs and self.outputs is a list. This must
        be called once before calling self.log_problem().
        
        Args:
            openmdao_problem: an OpenMDAO problem.
        """
        model = openmdao_problem.model
        inputs = model.list_inputs(val=False, out_stream=None)
        outputs = model.list_outputs(val=False, out_stream=None)
        for var in inputs:
            var_name = var[0]
            self.inputs[var_name] = list()
        for var in outputs:
            var_name = var[0]
            self.outputs[var_name] = list()

    def log_problem(self, openmdao_problem):
        """ Logs problem variables.
        
        Args:
            openmdao_problem: an OpenMDAO problem. Must have same input
                and output values as openmdao_problem passed to 
                init_problem().
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
        
        Returns:
            Dictionary with keys ['inputs', 'outputs']. Each contains a
            dataframe object corresponding to the key. See 
            _flatten_dict() for format of column names.
        """
        df_log = {}
        flat_inputs, flat_outputs = self._flatten_log()
        df_log['inputs'] = pd.DataFrame(flat_inputs)
        df_log['outputs'] = pd.DataFrame(flat_outputs)
        return df_log

    def plot_inputs(self, x_axis_key):
        """ Plots inputs dataframe against x_axis_key values. 
        
        Args:
            x_axis_key: Key in inputs dataframe log. Values of this key
                becomes the x-axis of the plot.
        """
        df_log = self.dataframe_log()
        inputs = df_log['inputs']
        t = inputs[x_axis_key]
        input_keys = list(inputs.columns)
        input_keys.remove(x_axis_key)
        return plot_vars(inputs, t, columns=4, keys=input_keys)

    def plot_outputs(self, x_axis_key):
        """ Plots outputs dataframe against x_axis_key values.
        
        Args:
            x_axis_key: Key in inputs dataframe log. Values of this key
                becomes the x-axis of the plot.
        """
        df_log = self.dataframe_log()
        t = df_log['inputs'][x_axis_key]
        return plot_vars(df_log['outputs'], t, columns=4)

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

        Will go deeper into dictionary, building the flattened key name
        along the way, until it reaches a list. If the elements of the
        list are dicts (each with the same keys), then the list is 
        converted into a dictionary of lists and the program iterates
        into this dict. Otherwise, if the elements of the list are not
        dicts, the function obtains the new keys and values from the 
        list.

        Example:
        The dictionary...
        {        
            outputs: {
                '_debug': [
                    {'r': 100, 'x': [100, 0]},
                    ...
                    {'r': 200, 'x': [0, 200]}
                ]
            }
        }
        ...becomes the dictionary...
        {
            'outputs._debug.r': [100, ..., 200],
            'outputs._debug.x[0]': [100, ..., 0],
            'outputs._debug.x[1]': [0, ..., 200]
        }
        
        
        Args:
            var_name: Key associated with var_val. 
                Set to '' if this is first entry into _flatten_dict.
            var_val: Value associated with var_name.

        Returns:
            dictionary with depth of 1.
        """
        new_table = {}
        if isinstance(var_val, dict):
            for key, val in var_val.items():
                if var_name != '':
                    new_var_name = var_name + "." + key
                else:
                    new_var_name = key
                out_table = self._flatten_dict(new_var_name, val)
                new_table.update(out_table)
        elif isinstance(var_val, list):
            if isinstance(var_val[0], dict):
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
        for key, val in lod[0].items():
            dol[key] = []

        # invert list of dicts to dict of lists
        for dict_element in lod:
            for key, val in dict_element.items():
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


class GuidanceLog:
    """ Log used by GuidanceBase subclasses with OpenMDAO problems.
    
    Attributes:
        problem: OpenMDAOProblemLog object.
        thrust_cmd: List of floats, representing thrust commands.
    """
    problem: OpenMDAOProblemLog
    thrust_cmd: list[float]

    def __init__(self):
        self.problem = OpenMDAOProblemLog()
        self.thrust_cmd = []

    def init_problem(self, openmdao_problem):
        """ Initializes the problem object.
        
        Args:
            openmdao_problem: an OpenMDAO problem.
        """
        self.problem.init_problem(openmdao_problem)

    def log_guidance(self, openmdao_problem, thrust_cmd):
        """ Logs guidance values.

        Args:
            openmdao_problem: an OpenMDAO problem. Must have same input
                and output values as openmdao_problem passed to 
                init_problem().
            thrust_cmd: float, the commanded thrust.
        """
        self.problem.log_problem(openmdao_problem)
        self.thrust_cmd.append(thrust_cmd)

    def save_pkl(self, save_path):
        """ Convenience function for saving object as pickle file.
        
        Args:
            save_path: path-like object, save location for pickle file.
        """
        with open(save_path, 'wb') as fh:
            pkl.dump(self, fh)


class LogAnalyzer:
    """ Class for processing and plotting guidance and simulation logs.
    
    Attributes:
        guidance_log: GuidanceLog
        sim_log: SimulationLog
        config: Config
    """
    guidance_log: GuidanceLog
    sim_log: SimulationLog
    config: Config

    # for get_derived_values() and get_error_values()
    # TODO: remove target lan and inc values, extract from config.
    _target_lan: float
    _target_inc: float

    _shared_derived_values_function: typing.Callable
    _error_values_function: typing.Callable
    _final_error_values_function: typing.Callable
    _final_estimated_T_function: typing.Callable

    def __init__(self, config, guidance_log, sim_log):
        """ Initializes log analyzer
        
        Args:
            config: Config object, used to initialize guidance_log and 
                sim_log.
            guidance_log: GuidanceBase subclass. Must be contained in 
                sim_log.
            sim_log: SimulatorBase subclass.
        """
        self.guidance_log = guidance_log
        self.sim_log = sim_log
        self.config = config

        # Change log function depending on guidance method.
        # Using this instead of subclassing in the case that functions
        # are dependent on both the guidance method and simulation
        # class.
        if config.orbit_targeting_ascent:
            self._set_analysis_attributes({
                'target_lan': config.orbit_targeting_ascent.longitude_of_ascending_node,
                'target_inc': config.orbit_targeting_ascent.inclination,
                'shared_derived_values_function': self._shared_derived_values_orbit_targeting_ascent,
                'error_values_function': self._error_values_orbit_targeting_ascent,
                'final_error_values_function': self._final_error_values_orbit_targeting_ascent,
                'final_estimated_T': self._final_estimated_T_orbit_targeting_ascent
            })
        elif config.debug_ascent_1:
            self._set_analysis_attributes({
                'target_lan': config.debug_ascent_1.longitude_of_ascending_node,
                'target_inc': config.debug_ascent_1.inclination,
                'shared_derived_values_function': self._shared_derived_values_debug_ascent_1,
                'error_values_function': self._error_values_debug_ascent_1,
                'final_error_values_function': self._final_error_values_debug_ascent_1,
                'final_estimated_T': self._final_estimated_T_debug_ascent_1
            })
        else:
            raise NotImplementedError("No recognized guidance method " +
                "found in config.")

    def _set_analysis_attributes(self, input_dict):
        """ Sets attributes that depend on guidance method. 
        
        Using this function ensures that every required attribute is set
        for each guidance method. 

        Args:
            input_dict: dict containing attribute values.
            
        """
        self._target_lan = input_dict['target_lan']
        self._target_inc = input_dict['target_inc']
        self._shared_derived_values_function = input_dict['shared_derived_values_function']
        self._error_values_function = input_dict['error_values_function']
        self._final_error_values_function = input_dict['final_error_values_function']
        self._final_estimated_T_function = input_dict['final_estimated_T']

    def get_derived_values(self, t_interp=None):
        """ Useful values calculated from the state log exclusively.
        
        Args:
            t_interp: [s] Array of time values to interpolate state at.

        Returns:
            Dataframe of derived values.
            
        """
        if t_interp is None:
            t = self.sim_log.state.get_time()
            pos = self.sim_log.state.get_position()
            vel = self.sim_log.state.get_velocity()
        else:
            t, pos, vel, _ = self.get_interpolated_state(t_interp)

        mu = self.config.body.gravitational_parameter
        target_lan = self._target_lan
        target_inc = self._target_inc

        derived = {}
        derived['t'] = t
        derived['dt'] = log_utils.get_time_steps(t)

        derived['radius'] = log_utils.get_radius(pos)
        derived['r_dot'] = log_utils.get_r_dot(pos, vel)
        derived['r_dot_dot'] = log_utils.get_r_dot_dot(t, pos, vel)

        derived['y1'] = log_utils.get_target_normal_position(pos, target_lan, target_inc)
        derived['y1_dot'] = log_utils.get_target_normal_velocity(vel, target_lan, target_inc)
        derived['y1_dot_dot'] = log_utils.get_target_normal_acceleration(t, vel, target_lan, target_inc)

        derived['v_theta'] = log_utils.get_v_theta(pos, vel)
        derived['a_theta'] = log_utils.get_a_theta(t, pos, vel)
        derived['non_gravity_acc_mag'] = log_utils.get_non_gravity_acc_mag(t, pos, vel, mu)
        derived['thrust_pitch'] = log_utils.get_thrust_pitch(t, pos, vel, mu)

        oe = log_utils.get_orbital_elements(pos, vel, mu)
        derived['semi_major_axis'] = oe[0, :]
        derived['ecc'] = oe[1, :]
        derived['inc'] = oe[2, :]
        derived['lan'] = oe[3, :]
        derived['argp'] = oe[4, :]
        derived['nu'] = oe[5, :]

        # a = 1/2 * (r_p + r_a)
        # e = 1 - r_p/a
        derived['pe'] = (1 - derived['ecc'])*derived['semi_major_axis']
        derived['ap'] = 2*derived['semi_major_axis'] - derived['pe']

        return pd.DataFrame(derived)

    def get_shared_derived_values(self, t_interp=None):
        """ Derived values dependent on both guidance and state logs. 
        
        Args:
            t_interp: Optional. If not None, must be an iterable of
            times at which to interpolate the state variables before
            finding derived values.

        Returns:
            Dataframe, where keys are names of derived values and 
            values are arrays of data.

        """
        return self._shared_derived_values_function(t_interp)

    def get_error_values(self):
        """ Dataframe of error between predicted and actual values. 
        
        Error values are the difference between values predicted by 
        guidance and actual values derived from the recorded state.

        Returns:
            Dataframe with independent variable column t and several
            error columns.
        """
        return self._error_values_function()

    def get_final_error_values(self):
        """ Dataframe of error between actual value at given time and
        the target final value.

        Returns:
            Dataframe with independent variable column t and several
            error columns.
        """
        return self._final_error_values_function()

    def get_final_estimated_T(self):
        """ Returns predicted guidance cutoff time.
        
        Returns:
            T [s], a float
        """
        return self._final_estimated_T_function()

    def get_final_error_table(self):
        """ Dictionary of error between actual and target values at 
        guidance cutoff.
        
        Returns:
            Dictionary with independent variable key t and several keys
            of errors. Value of each key is a float with error at time
            t. 
        """
        df_final_err = self.get_final_error_values()
        T = self.get_final_estimated_T()
        final_err_table = interp_table(T, 't', df_final_err)
        return final_err_table

    def get_interpolated_state(self, t_interp):
        """ Interpolates state log at given time.
        
        Returns:
            4-element tuple: (t, pos, vel, mass). t [s] is interpolated
            time, pos [m] is length 3 array of position, vel [m/s] is 
            length 3 array of velocity, mass [kg] is a float.
        """
        t = self.sim_log.state.get_time()
        pos = self.sim_log.state.get_position()
        vel = self.sim_log.state.get_velocity()
        mass = self.sim_log.state.get_mass()

        t_orig = t
        t = t_interp
        pos = np.array([np.interp(t_interp, t_orig, pos[i, :]) for i in range(pos.shape[0])])
        vel = np.array([np.interp(t_interp, t_orig, vel[i, :]) for i in range(vel.shape[0])])
        mass = np.interp(t_interp, t_orig, mass)

        return t, pos, vel, mass

    def plot_derived(self):
        """ Plots values calculated from state logs.
        
        Returns:
            2-tuple containing plot figure and axes.
        """
        df_derived = self.get_derived_values()
        t = df_derived['t']
        y_vars = list(df_derived.columns)
        y_vars.remove('t')
        fig, axs = plot_vars(df_derived, t, columns=4, keys=y_vars)
        fig.suptitle("Derived State Values")
        return fig, axs

    def plot_shared_derived(self):
        """ Plots values calculated from both the guidance and state
        logs.
        
        Returns:
            2-tuple containing plot figure and axes.
        """
        df_shared = self.get_shared_derived_values()
        t = df_shared['t']
        y_vars = list(df_shared.columns)
        y_vars.remove('t')
        fig, axs = plot_vars(df_shared, t, columns=4, keys=y_vars)
        fig.suptitle("Derived Guidance/State Values")
        return fig, axs

    def plot_error(self):
        """ Plots the error between predicted and actual values.
        
        Returns:
            2-tuple containing plot figure and axes.
        """
        df_err = self.get_error_values()
        t = df_err['t']
        y_vars = list(df_err.columns)
        y_vars.remove('t')
        fig, axs = plot_vars(df_err, t, columns=4, keys=y_vars)
        fig.suptitle("Guidance vs. Sim Error Values")
        return fig, axs

    def plot_final_error(self, tspan=None):
        """ Plots the error based on actual state and final target value.
        
        Args:
            tspan: [s] float, length of x-axis when focusing plot around
                final estimated time.
                
        Returns:
            2-tuple containing plot figure and axes.
        """
        df_final_err = self.get_final_error_values()
        T = self.get_final_estimated_T()
        # t_meco = self._get_commanded_meco()
        plotkwargs=None
        if tspan:
            df_final_err = df_final_err[(df_final_err['t'] >= T-tspan/2) &
                                        (df_final_err['t'] <= T+tspan/2)]
            plotkwargs={'marker': 'o'}
        t = df_final_err['t']
        y_vars = list(df_final_err.columns)
        y_vars.remove('t')
        fig, axs = plot_vars(df_final_err, t, columns=4, keys=y_vars, plotkwargs=plotkwargs)
        for ax1 in axs:
            for ax in ax1:
                ax.axvline(T, color='red', ls='-.', alpha=0.5)
        fig.suptitle("Final Error")
        return fig, axs

    def plot_inputs(self):
        """ Plots guidance inputs. See OpenMDAOProblemLog.plot_inputs().
        
        Returns:
            2-tuple containing plot figure and axes.
        """
        fig, axs = self.guidance_log.problem.plot_inputs('pitch_heading_query.query_t')
        fig.suptitle("Guidance Inputs")
        return fig, axs

    def plot_outputs(self):
        """ Plots guidance outputs. See OpenMDAOProblemLog.plot_outputs().
        
        Also plots the guidance thrust command across the guidance
        cutoff time.

        Returns:
            2-tuple containing plot figure and axes.
        """
        fig, axs = self.guidance_log.problem.plot_outputs('pitch_heading_query.query_t')
        fig.suptitle("Guidance Outputs")

        df_prob = self.guidance_log.problem.dataframe_log()
        _, ax2 = matplotlib.pyplot.subplots()
        t = df_prob['inputs']['pitch_heading_query.query_t']
        T = self.get_final_estimated_T()
        tspan = 0.5
        final_indices = (t>=T-tspan/2) & (t<=T+tspan/2)
        ax2.plot(t[final_indices], np.array(self.guidance_log.thrust_cmd)[final_indices], marker='o')
        ax2.axvline(T, color='red', ls='-.', alpha=0.5)
        ax2.set_ylabel("thrust_cmd")
        ax2.set_xlabel("t")
        ax2.set_title("Guidance Thrust Command")
        return fig, axs

    def plot_state(self):
        """ Plots the state logs.

        Returns:
            2-tuple containing plot figure and axes."""
        fig, axs = self.sim_log.state.plot_state()
        fig.suptitle("Guidance State")
        return fig, axs

    def save_csv(self, save_path):
        """ Saves dataframes as csv files.
        
        Args:
            save_path: path-like object, save directory for csv files.
        """
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        inputs_name = "inputs.csv"
        outputs_name = "outputs.csv"
        state_name = "state.csv"
        derived_name = "derived.csv"
        shared_derived_name = "shared_derived.csv"
        err_name = "err.csv"

        df_log = self.guidance_log.problem.dataframe_log()
        df_state = self.sim_log.state.dataframe_log()
        df_derived = self.get_derived_values()
        df_shared_derived = self.get_shared_derived_values()
        df_err = self.get_error_values()

        df_log['inputs'].to_csv(os.path.join(save_path, inputs_name))
        df_log['outputs'].to_csv(os.path.join(save_path, outputs_name))
        df_state.to_csv(os.path.join(save_path, state_name))
        df_derived.to_csv(os.path.join(save_path, derived_name))
        df_shared_derived.to_csv(os.path.join(save_path, shared_derived_name))
        df_err.to_csv(os.path.join(save_path, err_name))

        # TODO: This might break for the integrator
        vessel_name = "vessel.csv"
        df_vessel = self.sim_log.vessel.dataframe_log()
        df_vessel.to_csv(os.path.join(save_path, vessel_name))

    def _shared_derived_values_orbit_targeting_ascent(self, t_interp=None):
        derived = self._common_shared_derived_values()
        t_interp = derived['t']
        _, pos, _, _ = self.get_interpolated_state(t_interp)
        df_prob = self.guidance_log.problem.dataframe_log()
        target_lan_list = df_prob['inputs']['outer_loop.target_lan']
        target_inc_list = df_prob['inputs']['outer_loop.target_inc']
        target_argp = df_prob['inputs']['outer_loop.target_argp']

        ota_derived = {}
        ota_derived['projected_nu'] = log_utils.get_projected_true_anomaly(
            pos, target_lan_list, target_inc_list, target_argp)

        derived.update(ota_derived)
        return pd.DataFrame(derived)

    def _error_values_orbit_targeting_ascent(self):
        err_dict = self._common_error_values()
        return pd.DataFrame(err_dict)

    def _final_error_values_orbit_targeting_ascent(self):
        derived = self.get_derived_values()
        final_err_dict = {
            't': derived['t'],
            'dt': derived['dt'],
            'target_lan_err': (derived['lan'] - 
                               self.config.orbit_targeting_ascent.longitude_of_ascending_node),
            'target_inc_err': (derived['inc'] -
                               self.config.orbit_targeting_ascent.inclination),
            'target_ap_err': (derived['ap'] -
                              self.config.orbit_targeting_ascent.apoapsis),
            'target_pe_err': (derived['pe'] -
                              self.config.orbit_targeting_ascent.periapsis),
            'target_argp_err': (derived['argp'] -
                                self.config.orbit_targeting_ascent.argument_of_periapsis)
        }
        return pd.DataFrame(final_err_dict)

    def _final_estimated_T_orbit_targeting_ascent(self):
        df_prob = self.guidance_log.problem.dataframe_log()
        estimated_T_arr = df_prob['outputs']['outer_loop.T']
        return estimated_T_arr.iloc[-1]

    def _shared_derived_values_debug_ascent_1(self):
        derived = self._common_shared_derived_values()
        return pd.DataFrame(derived)

    def _error_values_debug_ascent_1(self):
        err_dict = self._common_error_values()
        return pd.DataFrame(err_dict)

    def _final_error_values_debug_ascent_1(self):
        derived = self.get_derived_values()
        final_err_dict = {
            't': derived['t'],
            'dt': derived['dt'],
            'target_lan_err': (derived['lan'] - 
                               self.config.debug_ascent_1.longitude_of_ascending_node),
            'target_inc_err': (derived['inc'] -
                               self.config.debug_ascent_1.inclination),
            'target_r_T_err': (derived['radius'] -
                           self.config.debug_ascent_1.radius),
            'target_r_dot_T_err': (derived['r_dot'] -
                           self.config.debug_ascent_1.radial_velocity)
        }
        return pd.DataFrame(final_err_dict)

    def _final_estimated_T_debug_ascent_1(self):
        T = self.config.debug_ascent_1.terminal_time
        return T

    def _common_shared_derived_values(self):
        """ Derived values common across all guidance methods. """
        df_prob = self.guidance_log.problem.dataframe_log()
        t_interp = np.unique(df_prob['inputs']['pitch_heading_query.query_t'])
        t, pos, vel, mass = self.get_interpolated_state(t_interp)

        df_prob = self.guidance_log.problem.dataframe_log()
        target_lan_list = df_prob['inputs']['outer_loop.target_lan']
        target_inc_list = df_prob['inputs']['outer_loop.target_inc']
        cmd_thrust_pitch = df_prob['outputs']['pitch_heading_query.cmd_pitch']
        cmd_thrust_yaw = df_prob['outputs']['pitch_heading_query.cmd_heading']
        F_thrust_max = df_prob['inputs']['outer_loop.v_e'][0] * df_prob['inputs']['outer_loop.m_dot'][0]
        thrust_acc_pcf = log_utils.get_thrust_acc_PCF(
            pos,
            cmd_thrust_pitch,
            cmd_thrust_yaw,
            mass,
            target_lan_list,
            target_inc_list, F_thrust_max)

        shared_derived = {}
        shared_derived['t'] = t
        shared_derived['dt'] = log_utils.get_time_steps(t)
        shared_derived['a_thrust_i_pcf'] = thrust_acc_pcf[0]
        shared_derived['a_thrust_j_pcf'] = thrust_acc_pcf[1]
        shared_derived['a_thrust_k_pcf'] = thrust_acc_pcf[2]

        theta_hat_pcf = log_utils.get_theta_hat_PCF(
            pos, vel, target_lan_list, target_inc_list
        )

        shared_derived['theta_hat_i_pcf'] = theta_hat_pcf[0]
        shared_derived['theta_hat_j_pcf'] = theta_hat_pcf[1]
        shared_derived['theta_hat_k_pcf'] = theta_hat_pcf[2]

        return shared_derived

    def _common_error_values(self):
        # NOTE: Should be dataframe, but currently oe is stored as array.
        df_prob = self.guidance_log.problem.dataframe_log()
        query_t = np.unique(df_prob['inputs']['pitch_heading_query.query_t'])
        df_deriv = self.get_derived_values(
            t_interp=query_t)
        input_prob_dict = {
            't': query_t,
            'dt': df_deriv['dt'],
            'r': df_prob['outputs']['pitch_heading_query._debug.r'],
            'r_dot': df_prob['outputs']['pitch_heading_query._debug.r_dot'],
            'r_dot_dot': df_prob['outputs']['pitch_heading_query._debug.r_dot_dot'],
            'y': df_prob['outputs']['pitch_heading_query._debug.y'],
            'y_dot': df_prob['outputs']['pitch_heading_query._debug.y_dot'],
            'y_dot_dot': df_prob['outputs']['pitch_heading_query._debug.y_dot_dot'],
            'a_thrust_mag': df_prob['outputs']['pitch_heading_query._debug.a_thrust_mag'],
            'cmd_pitch': df_prob['outputs']['pitch_heading_query.cmd_pitch']
        }

        err_dict = {
                't': df_deriv['t'],
                't_step': df_deriv['dt'],
                'r_err': (input_prob_dict['r'] - 
                    df_deriv['radius']),
                'r_dot_err': (input_prob_dict['r_dot'] - 
                    df_deriv['r_dot']),
                'r_dot_dot_err': (input_prob_dict['r_dot_dot'] - 
                    df_deriv['r_dot_dot']),
                'y1_err': (input_prob_dict['y'] - 
                    df_deriv['y1']),
                'y1_dot_err': (input_prob_dict['y_dot'] - 
                    df_deriv['y1_dot']),
                'y1_dot_dot_err': (input_prob_dict['y_dot_dot'] - 
                    df_deriv['y1_dot_dot']),
                'thrust_acc_err': (input_prob_dict['a_thrust_mag'] - 
                    df_deriv['non_gravity_acc_mag']),
                'thrust_alpha_err': (input_prob_dict['cmd_pitch'] - 
                    df_deriv['thrust_pitch'])
            }
        return err_dict

    def _get_commanded_meco(self):
        df_prob = self.guidance_log.problem.dataframe_log()
        query_t = np.unique(df_prob['inputs']['pitch_heading_query.query_t'])
        df_thrust_cmd = pd.DataFrame({
            't': query_t,
            'thrust_cmd': self.guidance_log.thrust_cmd
        })
        commanded_meco_index = np.where(df_thrust_cmd['thrust_cmd']==0)[0][0]
        #TODO:  WIP WIP
        return df_thrust_cmd[commanded_meco_index]
    