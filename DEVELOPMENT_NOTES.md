# Development Notes

Design decisions, known limitations, and coding conventions for this
project — read before making non-trivial changes. For a guided
explanation of what the model does, see `MODEL_WALKTHROUGH.md`.

## Ownership

- **Carlos**: master Simulink file, full implementation/integration,
  Simscape/HIL, STM32 firmware, PCB, CAD.
- **Umut**: mathematical modeling, controller architecture (LQR/DCM TVC,
  servo actuator dynamics). The controller subsystems are off-limits for
  others until the plant is considered done.

## Known limitations

- **Reference attitude (`DCM_ref`)**: three different values exist in the
  codebase; only the TVC controller's hardcoded one is actually live. Not
  yet consolidated into one source.
- **CG at burnout**: `x_cg_burnout` (1.037 m) is derived assuming
  propellant burns from the motor pivot, not the true propellant
  centroid — a simplifying assumption, not a measurement.
- **Inertia model**: `I(t)` is diagonal-only, with no parallel-axis
  correction for the CG shift. `Iyy`/`Izz` are measured (bifilar
  pendulum); `Ixx` is still an unmeasured placeholder. Client-confirmed
  acceptable for Milestone 1, but any controllability conclusion from the
  LQR design is only as good as this placeholder.
- **Aerodynamic drag**: not modeled (`F_aero = 0`). Client-agreed as
  negligible at the current flight envelope; revisit if speeds/altitudes
  increase.
- **Chaotic near touchdown**: tiny numerical differences (solver
  tolerance, an unrelated model edit) can produce meter-scale differences
  in apogee/touchdown, while ascent stays stable. When running the
  sensitivity sweep, evaluate metrics during ascent, not touchdown.
- **TVC allocation uses a static CG**: the controller's thrust-allocation
  matrix uses a fixed (t=0) moment arm, while the plant's actual
  force/moment mixing tracks the burning CG dynamically. Real asymmetry,
  not yet reconciled.
- **Descent is a placeholder**: descent motors run on a flat nominal
  thrust value; the descent thrust curve CSV is loaded but not wired in.
  Current focus is ascent only.
- **`matl.m`'s InitFcn requires the project root as MATLAB's working
  folder** at load/update time (see the README's Getting Started). Not
  fixed at the root cause — a future improvement would resolve the path
  relative to the model file itself instead of relying on cwd.
- **A few `rocket.*` parameters are unused or duplicated**:
  `gimbal_limit_ascent_deg`/`gimbal_limit_hover_deg` aren't wired to the
  controller (which hardcodes its own limits); `engine_pivot_x_from_cg`,
  `T_total_nominal`, `m_prop_ascent_each`/`descent_each`, and
  `burn_rate_each` are informational only, not read by any live block.

## Recent additions

- **`Simulation Monitoring` subsystem** (root level): one place with a
  scope per signal group — Position, Velocity, Attitude, Angular Rates,
  Thrust, Forces, Moments — for reviewing a run without hunting through
  the model. Per-block debug scopes elsewhere are unchanged.

## Physics / math conventions

- Full variable-inertia Euler equation:
  `M = I*omega_dot + I_dot*omega + omega x (I*omega)`. `I_dot*omega` is an
  explicit separate output from `MassInertiaModel`, not folded in.
- Attitude error is the vector part of a quaternion/DCM comparison, not
  Euler angle subtraction — avoids gimbal lock at the ~90° pitch launch
  attitude.
- No separate exhaust-velocity port; the thrust curve already encodes
  propulsive force.
- `x_b` is the vehicle's thrust axis (positive = up at the nominal
  near-vertical attitude). `h = -Xe(3)` (NED frame, `Xe(3)` positive-down).

## Style & conventions

- Every editable plant parameter lives in `matl.m`, never hardcoded
  inside a MATLAB Function/Stateflow block — Stateflow can't resolve
  `rocket.*` struct fields internally, so any block needing one takes it
  as an explicit input port. Applies to the plant; the controller is
  off-limits for now (see Ownership).
- `matl.m` and `lqr_gain_design.m` run as scripts sharing the base MATLAB
  workspace — always start a parameter script with `clearvars` so it
  can't silently inherit a stale variable from a previous run.
- Prefer native Simulink blocks over MATLAB Function blocks; reserve the
  latter for logic that genuinely needs it.
- No persistent variables in MATLAB Function blocks under continuous
  solvers.
- Modular architecture: gravity, thrust, drag, mass dynamics as separate
  blocks.
- Goto/From tags: never jump more than one subsystem level in a single
  tag — use a real Outport at each boundary and reserve Goto/From for the
  final, adjacent hop only. A mis-routed tag can silently compile to a
  frozen constant sample time instead of erroring; verify with
  `get_param(fromBlock,'CompiledSampleTime')` after adding a new pair.
- When a block's port order/count changes, rewire by signal name, never
  by raw port number.
- Build and test any new force/logic-summing block in isolation before
  wiring it into the live model.
- All code comments in English. No em-dashes.

## Solver

Fixed-step `ode4` at 0.001 s. Chosen after benchmarking against the
default `ode15s` and `ode45`, both far tighter than this model needs — no
accuracy loss observed.
