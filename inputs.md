# Input File Reference
The PROGRAM requires yaml files to run guidance. The required keys can be in the same file, or separate files, as long as all keys are received by the PROGRAM upon execution. The following keys are required:

- spacecraft
- body
- mission
- integrator | krpc

Either 'integrator' or 'krpc' may be defined, but not both. 'integrator' sets the guidance to run on an internal integrator. 'krpc' connects to a KRPC server in KSP and transmits guidance commands to the active spacecraft.

## Spacecraft 
Key                 | Units | Required  | Type   | Description
---                 | ---   | ---       | ---    | ---
specific_impulse    | s     | Yes       | `float`  | Specific impulse of engine in vacuum.
thrust              | N     | Yes       | `float`  | Maximum thrust of engine in vacuum.
wet_mass            | kg    | Yes       | `float`  | Total mass of vehicle at guidance start.

## Body
Key                     | Units     | Required  | Type  | Description
---                     | ---       | ---       | ---   | ---
gravitational_parameter | m^3/s^-2  | Yes       | `float` | Gravitational parameter of major body.

## Mission
Key                         | Units | Required  | Type      | Description
---                         | ---   | ---       | ---       | ---
apoapsis                    | m     | Yes       | `float`   | Target apoapsis radius, from major body center.
periapsis                   | m     | Yes       | `float`   | Target periapsis radius, from major body center.
longitude_of_ascending_node | deg.  | Yes       | `float`   |
inclination                 | deg.  | Yes       | `float`   | Range of [-90, 90).
argument_of_periapsis       | deg.  | Yes       | `float`   |

## Integrator
Uses internal integrator, intended for testing guidance.
Key                 | Units | Required  | Type      | Description
---                 | ---   | ---       | ---       | ---
simulation_end_time | s     | Yes       | `float`   | Length of time the simulation will run.
initial_position    | m     | Yes       | 1x3 `vector` [`float`]    | Position at start of guidance.
initial_velocity    | m/s   | Yes       | 1x3 `vector` [`float`]    | Velocity at start of guidance.

## KRPC
Connects to KRPC server for guidance.
Key         | Units | Required  | Type      | Description
---         | ---   | ---       | ---       | ---
name        | N/A   | Yes       | `string`  | Name of this program when displayed on KRPC server.
countdown   | s     | No        | `float`   | Sets guidance to occur `countdown` seconds after program connection to KRPC.

