import yaml
from enum import Enum
from pydantic import BaseModel, Field, PositiveFloat
import os

class GuidanceName(Enum):
    DEFAULT = "default"

class Spacecraft(BaseModel):
    specific_impulse: PositiveFloat
    thrust: PositiveFloat
    wet_mass: PositiveFloat

class Mission(BaseModel):
    gravitational_parameter: PositiveFloat
    apoapsis: PositiveFloat
    periapsis: PositiveFloat
    # Consider making this input in degrees and just
    # modulus it.
    true_anomaly: PositiveFloat

class Integrator(BaseModel):
    guidance: GuidanceName
    outer_loop_interval: PositiveFloat
    simulation_end_time: PositiveFloat
    # add check that directory exists
    log_path: str
    initial_position: list
    initial_velocity: list

class KSP_Interface(BaseModel):
    guidance: GuidanceName
    outer_loop_interval: PositiveFloat
    simulation_end_time: PositiveFloat
    # add check that directory exists
    log_path: str
    initial_position: list[float] = Field(min_length=3, max_length=3)
    initial_velocity: list[float] = Field(min_length=3, max_length=3)

class Config(BaseModel):
    spacecraft: Spacecraft
    mission: Mission
    # integrator: Integrator | None
    ksp_interface: KSP_Interface

def load_input(filenames):
    input_data = {}
    # Add functions for checking yaml input and if filename exists
    for filename in filenames:
        with open(filename, 'r') as fh:
            yaml_input = yaml.safe_load(fh)
        input_data.update(yaml_input)

    return input_data

def relative_path(filepath):
    return os.path.abspath(os.path.join(__file__, '..', filepath))

if __name__ == '__main__':
    input_filenames = [relative_path("../spacecraft/test_spacecraft.yaml"),
    relative_path("../scenarios/test_config.yaml")]
    input_data = load_input(input_filenames)
    config = Config(**input_data)
    print(config.spacecraft.thrust)
    