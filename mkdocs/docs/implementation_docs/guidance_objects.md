## 1. Guidance Objects

Guidance objects use config objects during initialization.

Guidance objects are objects that encapsulate guidance algorithms. They act as an interface between guidance implementations and simulation objects. Guidance objects are defined in `guidance_interface.py`.

`GuidanceBase` is the abstract base class that defines the interface for all guidance objects. All guidance objects subclass `GuidanceBase`. 

The most important member function of `GuidanceBase` is `get_command()`, which returns the commanded thrust, pitch, and heading, given the current state.

Guidance objects are used when initializing simulation objects.


### 1.1. OrbitTargetingAscent

Single-stage guidance algorithm that targets:

- apoapsis
- periapsis
- longitude of ascending node
- inclination
- argument of periapsis

This uses the `OrbitTargetingAscentGroup` model. The connections of the components within this model is shown in the following figure.


<figure>
    <img alt="PLACEHOLDER" src="../../img/OrbitTargetAscentChart.svg" style="width: 695px;" />
    <figcaption>
        Chart of interconnections in the OrbitTargetingAscentGroup object, omitting most user inputs.
    </figcaption>
</figure>

Note that the `OrbitGuidanceComponent` has a boolean input `"run_outer_loop"` (not labeled on the figure): When it is set to true, new values for $T$ and the $c_1,\, c_2$ constants are calculated when `.compute()` is called for this model. Otherwise, only the `EnginePropertyEstimator` and `PitchHeadingQuery` components are run. 

The rate of outer loop calls is set in the simulation object which contains this guidance object. Ideally, the outer loop would be run once every few seconds.

### 1.2. DebugAscent1

Single-stage guidance algorithm that targets:

- radius
- radial velocity
- cut-off time
- longitude of ascending node
- inclination

The purpose of this guidance object is to test a subset of the components used in `OrbitTargetingAscent`. See `test_integrator_sim.py` for these tests.

<figure>
    <img alt="PLACEHOLDER" src="../../img/DebugAscent1Chart.svg" style="width: 695px;" />
    <figcaption>
        Chart of interconnections in the DebugAscent1Group object, omitting most user inputs.
    </figcaption>
</figure>

Like the `OrbitTargetingAscent` guidance object, `VThetaSolverOuterLoop` has a boolean `run_outer_loop` input which determines if it will run and compute new values for $c_1,\, c_2$ when `.compute()` is called for this model.
