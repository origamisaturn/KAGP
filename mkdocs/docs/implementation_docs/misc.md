## Table of Contents

- [1. Guidance Objects](#1-guidance-objects)
    - [1.1. OrbitTargetingAscent](#11-orbittargetingascent)
    - [1.2. DebugAscent1](#12-debugascent1)
- [2. Simulation Objects](#2-simulation-objects)
    - [2.1. Integrator Sim](#21-integratorsim)
    - [2.2. KRPCClient](#22-krpcclient)
- [3. Guidance Components](#3-guidance-components)
    - [3.1. RadialYawGuidance](#31-radialyawguidance)
    - [3.2. TimeToGo](#32-timetogo)
    - [3.3. VThetaSolver](#33-vthetasolver)
    - [3.4. PitchHeadingQuery](#34-pitchheadingquery)
    - [3.5. OrbitTargeting](#35-orbittargeting)
    - [3.6. EnginePropertyEstimator](#36-enginepropertyestimator)
- [Appendix A: Symbols](#appendix-a-symbols)
- [Appendix B: Reference Frames](#appendix-b-reference-frames)
- [Appendix C: Abbreviated Derivation](#appendix-c-abbreviated-derivation)
    - [C.1. Fixed-Thrust Model](#c1-fixed-thrust-model)
    - [C.2. Generalized Guidance Law](#c2-generalized-guidance-law)
    - [C.3. Radial Guidance Law](#c3-radial-guidance-law)
    - [C.4. Plane Control Guidance Law](#c4-plane-control-guidance-law)
    - [C.5. Time-to-Go](#c5-time-to-go)
    - [C.6. Final Circumferential Velocity](#c6-final-circumferential-velocity)
    - [C.7. Pitch and Heading](#c7-pitch-and-heading)
    - [C.8. Final True Anomaly](#c8-final-true-anomaly)
    - [C.9. Orbit Targeting](#c9-orbit-targeting)
- [References](#references)
## Misc

All units are SI units, and angles are in radians, unless specified otherwise.

<!-- gcherry/

    - tests/ contains tests.
    - config.py: Defines configuration files.
    - guidance_components.py: Implementation of guidance algorithm components.
    - guidance_interface.py: Defines guidance objects.
    - log.py: Objects for logging simulation data.
    - log_utils.py: Utility functions for logging.
    - main_script.py: Command line script.
    - rk4.py: Numerical integrator.
    - sim_interface.py: Defines simulation objects.
    - transform.py: Functions for transforming between reference frames. -->


