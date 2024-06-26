import yaml
import os
from enum import Enum
from pydantic import (
    BaseModel, PositiveFloat, NonNegativeFloat, model_validator)
from typing import Optional
from typing_extensions import Self
from pathlib import Path


class GuidanceName(Enum):
    DEFAULT = "default"

class Spacecraft(BaseModel):
    specific_impulse: PositiveFloat
    thrust: PositiveFloat
    wet_mass: PositiveFloat

class CelestialBody(BaseModel):
    gravitational_parameter: PositiveFloat

class Mission(BaseModel):
    apoapsis: PositiveFloat
    periapsis: PositiveFloat
    longitude_of_ascending_node: NonNegativeFloat
    inclination: NonNegativeFloat
    argument_of_periapsis: NonNegativeFloat
    outer_loop_interval: PositiveFloat
    outer_loop_cutoff: PositiveFloat

class Integrator(BaseModel):
    simulation_end_time: PositiveFloat
    # add check that directory exists
    # log_path: str
    # TODO: Add check that this is a length 3 list non-zero
    initial_position: list[float]
    initial_velocity: list[float]

class KSP_Interface(BaseModel):
    simulation_end_time: PositiveFloat
    # add check that directory exists
    log_path: str

class Config(BaseModel):
    spacecraft: Spacecraft
    body: CelestialBody
    mission: Mission
    integrator: Optional[Integrator] = None
    ksp_interface: Optional[KSP_Interface] = None

    @model_validator(mode='after')
    def check_one_simulation_defined(self) -> Self:
        sim_attr = ['integrator', 'ksp_interface']
        defined_sum = 0
        for attr in sim_attr:
            if getattr(self, attr):
                defined_sum += 1
        if defined_sum == 0:
            raise ValueError("No simulation defined.")
        elif defined_sum != 1:
            raise ValueError("More than one simulation defined.")

def load_config(filenames):
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

def relative_path(filepath):
    return os.path.abspath(os.path.join(__file__, '..', filepath))

if __name__ == '__main__':
    input_filenames = [relative_path("../spacecraft/test_spacecraft.yaml"),
    relative_path("../scenarios/test_config.yaml")]
    config = load_config(input_filenames)
    print(config.spacecraft.thrust)
    