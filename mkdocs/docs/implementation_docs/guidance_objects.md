## 1. Guidance Objects

Guidance objects use config objects during initialization.

Guidance objects are objects that encapsulate guidance algorithms.  Guidance objects are defined in `guidance_interface.py`.

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

### 1.2. DebugAscent1

<figure>
    <img alt="PLACEHOLDER" src="../../img/DebugAscent1Chart.svg" style="width: 695px;" />
    <figcaption>
        Chart of interconnections in the DebugAscent1Group object, omitting most user inputs.
    </figcaption>
</figure>

