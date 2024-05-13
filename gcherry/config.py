import yaml
from enum import Enum
from pydantic import BaseModel, Field, PositiveFloat, NonNegativeFloat
import os

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
    log_path: str
    initial_position: list
    initial_velocity: list

class KSP_Interface(BaseModel):
    simulation_end_time: PositiveFloat
    # add check that directory exists
    log_path: str

class Config(BaseModel):
    spacecraft: Spacecraft
    body: CelestialBody
    mission: Mission
    # integrator: Integrator | None
    # ksp_interface: KSP_Interface

def load_config(filenames):
    input_data = {}
    # Add functions for checking yaml input and if filename exists
    for filename in filenames:
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
    