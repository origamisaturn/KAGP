# KAGP - Kerbal Ascent Guidance Procedure

KAGP is an ascent autopilot for single-stage spacecraft in Kerbal Space Program (KSP).

Docs:

- TODO: WEBSITE
- TODO: FOLDER MARKDOWN

This ascent program is based on the paper by G. Cherry referenced below. A derivation of the version of the ascent algorithm used for this program is provided in the online documentation (TODO: ADD HYPERLINK).

## Requirements

This program is intended to be used in KSP Realism Overhaul + Real Solar System. The algorithm may fail to converge in stock KSP due to the larger thrust-to-weight ratios.

Requires Python 3.10 due to reliance on poliastro. Recommend performing installation in a venv or using a Conda environment.

Requires [kRPC](https://github.com/krpc/krpc) to be installed for KSP.

## Install

Download the KAGP source files, then use `pip` to install the project. For example, with source files placed in folder `kagp`, and the current working directory being one level above, the installation command is:
```python
pip install ./kagp
```

## Use

KAGP requires a kRPC server with default settings running locally. The spacecraft must be ready for launch when the program is invoked. This program is only for single-stage ascents, it cannot perform staging.

Invoke the program with the `kagp` command. There are two subcommands:

- `kagp run`
- `kagp plotlog`

`kagp run` accepts a configuration file (LINK TO INPUTS.MD) and runs the ascent autopilot. See `examples/` for example config files.

Multiple config files can be provided. Config files are loaded in the order they are provided to the `kagp run` command. If a key is defined in multiple config files, the key in the latest config file takes priority.

Log files for a completed autopilot run are saved in `logs/`, in the current working directory. Each run is stored in a folder named with a timestamp.

`kagp plotlog` accepts the path to a log folder, and plots the logs.

## References

G. W. Cherry, "A General, Explicit, Optimizing Guidance Law for Rocket-Propelled Spaceflight," in <i>Astrodynamics Guidance and Control Conference, August 24-26, 1964, Los Angeles, CA, USA</i> [Online]. Available: ARC, https://arc.aiaa.org/doi/10.2514/6.1964-638