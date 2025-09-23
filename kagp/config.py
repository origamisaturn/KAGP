import pickle as pkl
from typing import Optional
from typing_extensions import Self
from pathlib import Path
from pydantic import (
    BaseModel, ValidationInfo,
    conlist, model_validator, field_validator,
    PositiveFloat, NonNegativeFloat)
import yaml
import numpy as np


class SpacecraftConfig(BaseModel):
    """ Spacecraft parameters for a single stage. """
    specific_impulse: PositiveFloat # s
    thrust: PositiveFloat # N
    wet_mass: PositiveFloat # kg


class CelestialBodyConfig(BaseModel):
    gravitational_parameter: PositiveFloat # m^3/s^2


class OrbitTargetingAscentConfig(BaseModel):
    apoapsis: PositiveFloat # m
    periapsis: PositiveFloat # m
    longitude_of_ascending_node: float # rad. [0, 2pi]
    inclination: NonNegativeFloat # rad. [0, pi]
    argument_of_periapsis: float # rad. [0, 2pi]

    enable_estimator: bool = True
    estimator_ignore_time: NonNegativeFloat = 5 # s
    estimator_output_time: NonNegativeFloat = 50 # s

    @field_validator('longitude_of_ascending_node', 'inclination',
                     'argument_of_periapsis')
    @classmethod
    def convert_deg_to_rad(cls, val: float, info: ValidationInfo) -> NonNegativeFloat:
        if isinstance(val, float):
            if info.field_name == 'inclination':
                assert val >= 0 and val <= 180, f'{info.field_name} must be between 0 and 180 degrees.'
                val = np.deg2rad(val%180)
            else:
                val = np.deg2rad(val%360)
        return val


class DebugAscent1Config(BaseModel):
    terminal_time: PositiveFloat # s
    radius: PositiveFloat # m
    radial_velocity: float # m/s
    longitude_of_ascending_node: float # rad. [0, 2pi]
    inclination: NonNegativeFloat # rad. [0, pi]

    @field_validator('longitude_of_ascending_node', 'inclination')
    @classmethod
    def convert_deg_to_rad(cls, val: float, info: ValidationInfo) -> NonNegativeFloat:
        if isinstance(val, float):
            if info.field_name == 'inclination':
                assert val >= 0 and val <= 180, f'{info.field_name} must be between 0 and 180 degrees.'
                val = np.deg2rad(val%180)
            else:
                val = np.deg2rad(val%360)
        return val


class IntegratorConfig(BaseModel):
    simulation_end_time: PositiveFloat # s
    initial_position: conlist(float, min_length=3, max_length=3) # [m, m, m]
    initial_velocity: conlist(float, min_length=3, max_length=3) # [m/s, m/s, m/s]
    # TODO: Check what happens when outer_loop_interval is zero.
    outer_loop_interval: NonNegativeFloat = 7 # s
    outer_loop_cutoff: NonNegativeFloat = 10 # s

    @model_validator(mode='after')
    def check_init_position_nonzero(self) -> Self:
        init_pos_mag = np.linalg.norm(self.initial_position)
        if init_pos_mag == 0:
            raise ValueError("initial_position is zero.")
        return self


class KRPCClientConfig(BaseModel):
    name: str = "KAGP"
    outer_loop_interval: PositiveFloat = 7 # s
    outer_loop_cutoff: PositiveFloat = 10 # s
    post_guidance_measurement: NonNegativeFloat = 5 # s
    main_engine_cutoff_shift: float = -0.061 # s


class Config(BaseModel):
    """ Main settings class.

    Contains all parameters to run ascent guidance in a simulation 
    environment.
    
    """
    spacecraft: SpacecraftConfig
    body: CelestialBodyConfig
    # Guidance method options
    orbit_targeting_ascent: Optional[OrbitTargetingAscentConfig] = None
    debug_ascent_1: Optional[DebugAscent1Config] = None
    # Simulator options
    integrator: Optional[IntegratorConfig] = None
    krpc_client: Optional[KRPCClientConfig] = None

    @model_validator(mode='after')
    def check_one_simulation_defined(self) -> Self:
        sim_attr = ['integrator', 'krpc_client']
        _assert_single_config_attr(self, sim_attr, "simulation")
        return self

    @model_validator(mode='after')
    def check_one_guidance_defined(self) -> Self:
        guidance_attr = ['orbit_targeting_ascent', 'debug_ascent_1']
        _assert_single_config_attr(self, guidance_attr, "guidance")
        return self

    def save_pkl(self, save_path):
        with open(save_path, 'wb') as fh:
            pkl.dump(self, fh)


def _assert_single_config_attr(config_model, attr_list, attr_kind):
    defined_sum = 0
    for attr in attr_list:
        if getattr(config_model, attr):
            defined_sum += 1
    if defined_sum == 0:
        raise ValueError("No {} attribute defined.".format(attr_kind))
    elif defined_sum != 1:
        raise ValueError("More than one {} attribute defined.".format(attr_kind))


def load_config(filenames):
    """ Creates a Config object based on input files. 
    
    Args:
        filenames: A list of .yaml files containing data in format
            specified by Config.

    Returns:
        A Config object.
    """
    input_data = {}
    for filename in filenames:
        p = Path(filename)
        if not (p.exists() and p.is_file()):
            raise RuntimeError("File '{}' does not exist.".format(p))
        with open(filename, 'r') as fh:
            yaml_input = yaml.safe_load(fh)
        input_data.update(yaml_input)

    config = Config(**input_data)
    return config
