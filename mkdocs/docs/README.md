![test-workflow](https://github.com/origamisaturn/gcherry/actions/workflows/github-actions-tests.yaml/badge.svg)
![pytest-coverage](coverage.svg)

# Single Stage Guidance
Implementation of single stage ascent "E-guidance" as described in [REFERENCE], with ability to control spacecraft in Kerbal Space Program (KSP).
Running in KSP requires the KRPC modification [LINK] to be installed in KSP.
[config inputs](inputs.md)
## Example
Invoke the program with the `gcherry` command. There are two subcommands:
`gcherry run` and `gcherry plotlog`.

`gcherry run` takes the path to yaml file(s) as input, and runs the guidance algorithm on either an internal integrator, or on a KSP spacecraft through a KRPC server. An invocation using the lunar ascent example is:
```
    gcherry run examples/ascent_lem.yaml examples/integrator_lunar_ascent.yaml 
```
Separating the spacecraft config file from the guidance config file, as is done in the example, allows swapping out different ascent vehicles for the same guidance path.

Config files are loaded and placed into a dictionary in the order they were provided to the command. If a key has already been provided by a preceeding config file when the next config file is loaded, the preceeding key is overwritten.

Log files for each run are saved in ./logs. Each run is stored in a folder named with a timestamp.

`gcherry plotlog` plots the logs, given the folder they are stored in.

```
    gcherry plotlog logs/071524_010721
```
It plots:
- state
- guidance inputs
- guidance outputs
- values derived from state
- values derived from guidance and state
- guidance error values
- final state error values

## Info
Implements ascent vehicle guidance
Mostly follows the book, implements yaw guidance and pitch guidance
less concern with computing power while attempting to follow the spirit
of the book. 
the vtheta solver is custom, uses RK4 instead of taylor series since it
would have been too complicated for little gain.
modified to have orbit targeting and estimation of engine parameters

The file specification for input can be found here:
examples are in this folder

see transform.py for frames used.

## Demo
A demo of the guidance in action can be found here: