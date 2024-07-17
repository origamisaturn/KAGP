# Input File Reference
The ascent program requires yaml configuration files to run. The config data can be defined in one file, or multiple files, as long as all necessary keys are passed to gcherry upon execution. The following keys are required:

- spacecraft
- body
- orbit_targeting_ascent | debug_ascent_1
- integrator | krpc_client

The '|' indicates that one, and only one, of the keys in the list must be defined.
Either 'integrator' or 'krpc_client' may be defined, but not both. 

## spacecraft
Key                 | Units | Required  | Type   | Description
---                 | ---   | ---       | ---    | ---
specific_impulse    | s     | Yes       | `float`  | Specific impulse of engine in vacuum.
thrust              | N     | Yes       | `float`  | Maximum thrust of engine in vacuum.
wet_mass            | kg    | Yes       | `float`  | Total mass of vehicle at guidance start.

## body
Key                     | Units     | Required  | Type  | Description
---                     | ---       | ---       | ---   | ---
gravitational_parameter | m^3/s^-2  | Yes       | `float` | Gravitational parameter of major body.

## orbit_targeting_ascent
Key                         | Units | Required  | Default   | Type      | Description
---                         | ---   | ---       | ---       | ---       | ---
apoapsis                    | m     | Yes       |           | `float`   | Target apoapsis radius, from major body center.
periapsis                   | m     | Yes       |           | `float`   | Target periapsis radius, from major body center.
longitude_of_ascending_node | deg   | Yes       |           | `float`   |
inclination                 | deg   | Yes       |           | `float`   | Range of [-90, 90).
argument_of_periapsis       | deg   | Yes       |           | `float`   |
enable_estimator            | N/A   | No        | `True`    | `bool`    | Enables engine property estimator. Calculates exhaust velocity and mass flow.
estimator_ignore_time       | s     | No        | 5.0       | `float`   | Will cause engine estimator to ignore thrust measurements until estimator_ignore_time seconds after guidance start. To avoid engine measurements during engine ramp-up.
estimator_output_time       | s     | No        | 50.0      | `float`   | Will start engine estimator calculations estimator_output_time seconds after guidance start. If too low, estimator may not converge.

## debug_ascent_1
Key                         | Units | Required  | Type      | Description
---                         | ---   | ---       | ---       | ---
terminal_time               | s     | Yes       | `float`   | Time at guidance termination.
radius                      | m     | Yes       | `float`   |
radial_velocity             | m/s   | Yes       | `float`   |
longitude_of_ascending_node | deg   | Yes       | `float`   |
inclination                 | deg   | Yes       | `float`   |

## integrator
Uses internal integrator, intended for testing guidance.

'integrator' sets the guidance to run on an internal integrator.
Key                         | Units | Required  | Default   | Type      | Description
---                         | ---   | ---       | ---       | ---       | ---
simulation_end_time         | s     | Yes       |           | `float`   | Length of time the simulation will run.
initial_position            | m     | Yes       |           | 1x3 `vector` [`float`]    | Position at start of guidance.
initial_velocity            | m/s   | Yes       |           | 1x3 `vector` [`float`]    | Velocity at start of guidance.
outer_loop_interval         | s     | No        | 7         | `float`   | Time between successive outer loop calculations.
outer_loop_cutoff           | s     | No        | 10        | `float`   | Outer loop calculation will be disabled outer_loop_cutoff seconds before the estimated terminal time.

## krpc_client
Connects to KRPC server for guidance.

'krpc_client' connects to a KRPC server in KSP and transmits guidance commands to the active spacecraft.

Key                         | Units | Required  | Default   | Type      | Description
---                         | ---   | ---       | ---       | ---       | ---
name                        | N/A   | No        | `gcherry` | `string`  | Name of this program when displayed on KRPC server.
outer_loop_interval         | s     | No        | 7         | `float`   | Time between successive outer loop calculations.
outer_loop_cutoff           | s     | No        | 10        | `float`   | Outer loop calculation will be disabled outer_loop_cutoff seconds before the estimated terminal time.
post_guidance_measurement   | s     | No        | 5         | `float`   | Client will continue to run and log data until post_guidance_measurement seconds after guidance termination.
main_engine_cutoff_shift    | s     | No        | -0.061    | `float`   | Client will send the thrust off command main_engine_cutoff_shift seconds after terminal time. Default is negative due to 0.060s lag time between client commanding thrust cutoff and KSP turning off engines.

