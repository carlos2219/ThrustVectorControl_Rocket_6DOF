# TVC Rocket 6DOF Simulation

A MATLAB/Simulink 6DOF flight simulation of a three-motor thrust-vector-control
(TVC) rocket burning KNSB propellant. Freelance project for client Kruthick
Jothimani, built with milestone-based, open-source-intent, "keep it simple"
design philosophy. Physical rocket parameters are not final; the client's
rocket is still under construction.

## Getting started

1. Open `rocket_upwork.slx` in Simulink.
2. **Set MATLAB's current working folder to the project root before opening
   or updating the model.** The model's `InitFcn` calls `matl;` (see
   `matl.m`), which only resolves with the project root as the working
   directory. See `NOTES.md` for the full writeup of this fragility.
3. Run/update the diagram as usual from there.

## Repository layout

- `rocket_upwork.slx` — the canonical master model (single source of
  truth for all simulation work).
- `matl.m` — the model's `InitFcn` source; builds the `rocket` struct
  (mass, geometry, CG/CP, propellant, thrust curves, LQR gains, etc.).
- `lqr_gain_design.m` — computes the LQR gain matrix used by the
  controller.
- `thrust_data.csv` / `thrust_data_ascent_clean.csv` /
  `thrust_data_descent_clean.csv` — static motor test data (raw and cleaned)
  used to build the ascent/descent thrust curves. **Current focus is ascent
  only:** the descent file is a placeholder (duplicate of the ascent data)
  pending real descent motor test data, and it is loaded into
  `rocket.descent_thrust_curve_t/N` but not yet wired into `PerMotorThrust`
  — descent motors still run on the flat `rocket.T_nominal` placeholder.
- `MODEL_WALKTHROUGH.md` — a guided, physical/logical-flow explanation of how
  the model works (plant, 6DOF integration, controller, actuator), meant to
  be read on its own without opening Simulink.
- `ENGINEERING_LOG.md` — design decisions, known gaps, and style
  conventions for this project. Read this before making non-trivial changes.
- `NOTES.md` — working notes (currently the `matl.m`/InitFcn working-directory
  fragility writeup).
- `ARCHIVE/` — retired files kept for historical reference only (the
  pre-pivot `SIM_model` lineage, a pristine snapshot of the model before
  active editing began, etc.). Not part of the active simulation.
- `initFcn.m` (root) — legacy, from a retired modeling approach; not executed
  by the current model.

## Requirements

- MATLAB and Simulink (developed against a recent release; see
  `ARCHIVE/rocket_upwork.slx.r2024b` for the last known-good R2024b save).
- Aerospace Blockset (6DOF integration block).
- Stateflow (MATLAB Function blocks used throughout the model).

## Status

Active development, Milestone 1, currently focused on the ascent stage only
(descent thrust is still a flat placeholder, see Repository layout above).
See `ENGINEERING_LOG.md`'s "Known gaps" section for the current list of
open items and simplifications.
