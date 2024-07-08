import yaml
import pickle as pkl
from enum import Enum
from pydantic import (
    BaseModel, conlist, PositiveFloat, NonNegativeFloat, model_validator)
from typing import Optional
from typing_extensions import Self
from pathlib import Path
import numpy as np


class GuidanceName(Enum):
    DEFAULT = "default"

class SpacecraftConfig(BaseModel):
    """ Spacecraft parameters for a single stage. """
    specific_impulse: PositiveFloat
    thrust: PositiveFloat
    wet_mass: PositiveFloat

class CelestialBodyConfig(BaseModel):
    gravitational_parameter: PositiveFloat

class OrbitTargetingAscentConfig(BaseModel):
    apoapsis: PositiveFloat
    periapsis: PositiveFloat
    longitude_of_ascending_node: NonNegativeFloat
    inclination: NonNegativeFloat
    argument_of_periapsis: NonNegativeFloat

class DebugAscent1Config(BaseModel):
    terminal_time: PositiveFloat
    radius: PositiveFloat
    radial_velocity: float
    longitude_of_ascending_node: NonNegativeFloat
    inclination: NonNegativeFloat

class IntegratorConfig(BaseModel):
    simulation_end_time: PositiveFloat
    initial_position: conlist(float, min_length=3, max_length=3)
    initial_velocity: conlist(float, min_length=3, max_length=3)
    # TODO: Check what happens when outer_loop_interval is zero.
    outer_loop_interval: NonNegativeFloat = 7
    outer_loop_cutoff: NonNegativeFloat = 10

    @model_validator(mode='after')
    def check_init_position_nonzero(self) -> Self:
        init_pos_mag = np.linalg.norm(self.initial_position)
        if init_pos_mag == 0:
            raise ValueError("initial_position is zero.")
        return self

class KRPCClientConfig(BaseModel):
    name: str = "PLACEHOLDER"
    outer_loop_interval: PositiveFloat = 7
    outer_loop_cutoff: PositiveFloat = 10

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
    # Add functions for checking yaml input and if filename exists
    for filename in filenames:
        p = Path(filename)
        if not (p.exists() and p.is_file()):
            raise RuntimeError("File '{}' does not exist.".format(p))
        with open(filename, 'r') as fh:
            yaml_input = yaml.safe_load(fh)
        input_data.update(yaml_input)

    config = Config(**input_data)
    return config