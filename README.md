# TVC Rocket 6DOF Simulation

A MATLAB/Simulink 6DOF flight simulation of a three-motor thrust-vector-control
(TVC) rocket burning KNSB propellant. Freelance project for client Kruthick
Jothimani, built with milestone-based, open-source-intent, "keep it simple"
design philosophy. Physical rocket parameters are not final; the client's
rocket is still under construction.

## Getting started

1. Open `UMUT/rocket_upwork.slx` in Simulink.
2. **Set MATLAB's current working folder to `UMUT/` before opening or
   updating the model.** The model's `InitFcn` calls `matl;` (see
   `UMUT/matl.m`), which only resolves with `UMUT/` as the working directory.
   See `NOTES.md` for the full writeup of this fragility.
3. Run/update the diagram as usual from there.

## Repository layout

- `UMUT/rocket_upwork.slx` — the canonical master model (single source of
  truth for all simulation work).
- `UMUT/matl.m` — the model's `InitFcn` source; builds the `rocket` struct
  (mass, geometry, CG/CP, propellant, thrust curves, LQR gains, etc.).
- `UMUT/lqr_gain_design.m` — computes the LQR gain matrix used by the
  controller.
- `UMUT/thrust_data.csv` / `UMUT/thrust_data_clean.csv` — static motor test
  data (raw and cleaned) used to build the ascent thrust curve.
- `MODEL_WALKTHROUGH.md` — a guided, physical/logical-flow explanation of how
  the model works (plant, 6DOF integration, controller, actuator), meant to
  be read on its own without opening Simulink.
- `CLAUDE.md` — the engineering log: design decisions, known gaps,
  verification notes, and style conventions for this project. Read this
  before making non-trivial changes.
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

Active development, Milestone 1. See `CLAUDE.md`'s "Known gaps" section for
the current list of open items and simplifications.
