![test-workflow](https://github.com/origamisaturn/gcherry/actions/workflows/github-actions-tests.yaml/badge.svg)
![pytest-coverage](coverage.svg)

# Single Stage Guidance
Implementation of single stage ascent "E-guidance" as described in [REFERENCE], with ability to control spacecraft in Kerbal Space Program (KSP).
Running in KSP requires the KRPC modification [LINK] to be installed in KSP.
[config inputs](inputs.md)
## Example
```
    gcherry run examples/newScriptKRPC2.yaml --log
    gcherry plotlog logs/071524_010721
```
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

## Demo
A demo of the guidance in action can be found here: