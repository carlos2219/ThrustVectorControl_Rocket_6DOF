# TVC Rocket — 6DOF Flight Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-Simulink-orange)
![Status](https://img.shields.io/badge/status-active%20development-blue)
![Focus](https://img.shields.io/badge/focus-ascent-lightgrey)

A 6DOF flight simulator for a three-motor thrust-vector-controlled (TVC)
rocket, built in MATLAB/Simulink. LQR/DCM attitude control, live mass/
inertia tracking, and closed-loop guidance from liftoff through landing.

Freelance build for client Kruthick Jothimani — milestone-based,
open-source-intent, "keep it simple" design. Physical rocket parameters
are not final; the real hardware is still under construction.

## How it works

```mermaid
flowchart LR
    THRUST["Thrust<br/>(per-motor thrust curves +<br/>flight-phase state machine)"]
    CONTROLLER["Controller<br/>(LQR attitude + descent<br/>throttle + lateral guidance)"]
    ROCKET["Rocket<br/>(servo actuators + force/moment<br/>mixer + 6DOF integration)"]

    THRUST -->|T_per_engine| CONTROLLER
    THRUST -->|T_per_engine| ROCKET
    CONTROLLER -->|alpha, beta| ROCKET
    ROCKET -->|attitude, rate,<br/>position, velocity| CONTROLLER
    ROCKET -->|height, velocity| THRUST

    ROCKET -.-> FT[Flight Termination]
    ROCKET -.-> MON[Simulation Monitoring]
    ROCKET -.-> ANIM[Animation]
```

Solid arrows are the closed control loop; dashed arrows are read-only
consumers (safety logic, scopes, 3D viewer) that don't feed anything
back. Full block-by-block detail lives in `MODEL_WALKTHROUGH.md`.

## Quick start

1. Open `rocket_upwork.slx` in Simulink.
2. Set MATLAB's current folder to the project root **first** — the
   model's `InitFcn` runs `matl.m`, which won't resolve otherwise.
3. Run or update the diagram as usual.

## What's inside

| File | Purpose |
|---|---|
| `rocket_upwork.slx` | canonical master model |
| `matl.m` | single source of truth for every plant parameter |
| `lqr_gain_design.m` | offline LQR gain design |
| `thrust_data_*_clean.csv` | motor thrust curves (descent is still a placeholder — see Status) |
| `MODEL_WALKTHROUGH.md` | guided tour of the model, block by block |
| `DEVELOPMENT_NOTES.md` | design decisions, known limitations, conventions |

## Requirements

- MATLAB + Simulink (recent release)
- Aerospace Blockset
- Stateflow

## Status

Milestone 1, ascent-focused. Descent thrust is still a flat placeholder
pending real motor test data. See `DEVELOPMENT_NOTES.md`'s "Known
limitations" for the full list of open items.
