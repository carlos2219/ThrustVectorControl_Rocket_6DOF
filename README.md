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
   directory — otherwise every `Constant` block that reads `rocket.*` will
   error with "Invalid setting for parameter 'Value'".
3. Run/update the diagram as usual from there.

## Repository layout

- `rocket_upwork.slx` — the canonical master model (single source of
  truth for all simulation work).
- `matl.m` — the model's `InitFcn` source; builds the `rocket` struct
  (mass, geometry, CG/CP, propellant, thrust curves, LQR gains, etc.).
- `lqr_gain_design.m` — computes the LQR gain matrix used by the
  controller.
- `thrust_data_ascent_clean.csv` / `thrust_data_descent_clean.csv` —
  cleaned static motor test data used to build the thrust curves.
  **Current focus is ascent only:** the descent file is a placeholder
  (duplicate of the ascent data) pending real descent motor test data —
  descent motors still run on a flat nominal-thrust placeholder.
- `MODEL_WALKTHROUGH.md` — a guided, physical/logical-flow explanation of how
  the model works (plant, 6DOF integration, controller, actuator), meant to
  be read on its own without opening Simulink.
- `DEVELOPMENT_NOTES.md` — design decisions, known limitations, and coding
  conventions for this project. Read this before making non-trivial changes.

## Requirements

- MATLAB and Simulink (a recent release).
- Aerospace Blockset (6DOF integration block).
- Stateflow (MATLAB Function blocks used throughout the model).

## Status

Active development, Milestone 1, currently focused on the ascent stage only
(descent thrust is still a flat placeholder, see Repository layout above).
See `DEVELOPMENT_NOTES.md`'s "Known limitations" section for the current
list of open items and simplifications.
