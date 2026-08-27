# TVC Rocket 6DOF Simulation — Engineering Log

## Project overview
Freelance MATLAB/Simulink project (client: Kruthick Jothimani). 6DOF flight
sim of a three-motor TVC rocket burning KNSB propellant. Client philosophy:
simplicity over completeness, milestone-based, open-source intent. Physical
parameters are not final (rocket still under construction).

Client's end goal: one clean, abstracted plant block, decoupled from the
controller, so different control methodologies can be tested against it.
Current focus is the plant (mass/inertia/thrust/CG) and the ascent stage
only; the controller (Umut's TVC/servo subsystems) stays untouched until the
plant is considered done.

## Repository layout
Everything lives flat at the project root (no `UMUT/` subfolder anymore).

- `rocket_upwork.slx` — canonical master model, all active work happens here.
- `matl.m` — InitFcn source (model's InitFcn is the bare string `matl;`,
  only resolves if MATLAB's cwd is the project root, see `NOTES.md`). Single
  source of truth for every editable plant parameter. Runs `clearvars` first
  so it never depends on leftover workspace state (item 17).
- `lqr_gain_design.m` — LQR gain matrix, invoked from `matl.m`.
- `initFcn.m` — legacy, not executed by the current model.
- `NOTES.md` — cwd-fragility writeup.
- `ARCHIVE/` — retired files (pre-pivot SIM_model lineage, superseded
  backups); historical reference only, not part of the active sim.
- `thrust_data.csv` — raw static motor test data, kept for traceability,
  not read directly by `matl.m`.
- `thrust_data_ascent_clean.csv` — cleaned ascent thrust curve, loaded in
  full (no truncation); `t_burn_ascent` is derived from its last timestamp.
- `thrust_data_descent_clean.csv` — placeholder (duplicate of the ascent
  data) for future real descent motor data; loaded but not yet wired into
  the thrust computation, since ascent is the current focus.

## Ownership split
- Carlos: master Simulink file, full implementation/integration, Simscape/HIL,
  STM32 firmware, PCB, CAD.
- Umut: mathematical modeling, controller architecture (LQR/DCM TVC, servo
  actuator dynamics).

## Legacy: SIM_model (archived)
Carlos's original plant model, retired once `MassInertiaModel(t)` was ported
into `rocket_upwork.slx`. One piece not yet ported: `DragForce`
(`F_drag = -0.5*rho*|v|^2*Cd*A_ref*v_hat`). Parameter convention going
forward: struct fields in Constant block Values (`rocket.Cd` etc.), not flat
workspace variables.

## Known gaps and decisions
1. **6DOF block**: `aerolib6dof2/Custom Variable Mass 6DOF (Quaternion)`
   (migrated from a legacy masked block), `mtype=Custom Variable`, quaternion
   state to avoid gimbal lock at 90° pitch. Verified equivalent to the old
   block (max diff ~2e-11 m position under a fixed-step isolation test).
   Gotcha: the `q` output sits at port 4, shifting `DCMbe`/`Vb`/`omega_be`
   down one index vs. the old block — always wire/verify by signal name,
   never by port number.
2. **DCM_reference mismatch**: informational, not resolved. Three different
   `DCM_ref` values exist in the codebase; only the TVC controller's
   hardcoded one is actually live.
3. **x_cg(t) integration**: `MassInertiaModel(t)` -> `AssembleRCG` -> dynamic
   `r_cg` (row 1 axial, rows 2-3 static lateral) -> `rocket_forces_moments`.
   `r_cg(1,i) = x_cg - engine_pivot_from_nose`.
4. **`r_cg_t` removed**, doesn't exist. Stateflow requires every declared
   output be assigned, so the old unused `r_cg_t` output was dropped.
   `AssembleRCG` is the one live mechanism for the dynamic moment arm —
   don't reintroduce `r_cg_t` as a second path for the same thing.
5. `rocket.x_cg_initial = 1.090 m` (measured, balancing, fully fueled).
   `rocket.x_cg_burnout = 1.0373 m` (derived, assumes propellant burns from
   the motor pivot — a simplifying assumption, not the true propellant
   centroid).
6. eML/Stateflow chart scripts cannot resolve `rocket.*` struct fields
   internally (`Stateflow:cdr:ErrorsParsingEmlFcn`). Any MATLAB Function
   block needing a `rocket.*` value takes it as an explicit input port fed
   by a Constant block, never reads the struct directly.
7. **Duplicate parameters, resolved for `MassInertiaModel`**: `m_dry`,
   `m_prop_total`, `mass_burn_duration`, `I_dry`, `I_prop` are explicit
   Constant-block inputs sourced from `matl.m`, no local hardcoding left.
   `mass_burn_duration` (=20s) is intentionally independent from
   `t_burn_ascent`/`t_burn_descent` (one continuous depletion model, not two
   separate burns) — not silently merged.
8. `I(t)` is diagonal-only, no parallel-axis correction for the CG shift —
   a deliberate M1 simplification, client-confirmed acceptable as long as
   every parameter lives in `matl.m` (item 7). `Iyy`/`Izz` measured (bifilar
   pendulum); `Ixx` is still an unmeasured placeholder.
9. Variable mass/inertia wired into the 6DOF block. `vre_flag` is on,
   `Vre` fed `[0;0;0]` (no separate exhaust-velocity modeling); `m_dot` is
   wired to the analytic output since the block needs it internally even
   with `Vre=0`. Lesson learned here: a Goto/From tag jumping more than one
   subsystem level can silently compile to a frozen constant sample time
   instead of erroring — see the routing rule under Style below.
10. **cwd fragility**: `matl.m`'s InitFcn only resolves if MATLAB's cwd is
    the project root at load/update time. Not fixed (still a bare `matl;`
    call, no explicit path) — see `NOTES.md`.
11. `rocket.t_burn_ascent` is derived from `thrust_data_ascent_clean.csv`'s
    own last timestamp (currently ~21.5s), not a manually chosen number —
    updates automatically if the CSV changes.
12. **Per-motor thrust**: `PerMotorThrust` interpolates the real ascent
    thrust curve per motor (`thrust_pert`/`ignition_delay` support per-motor
    sensitivity sweeps later, zero by default). Ascent curve is used in
    full, no longer truncated. Descent motors still use the flat
    `rocket.T_nominal` placeholder — current focus is ascent only;
    `thrust_data_descent_clean.csv` is loaded but not wired in yet. Ascent
    data is raw scale readings, not corrected for motor mass loss during
    the burn.
13. **Launch-pad ground reaction**: `GroundReaction` cancels the net-downward
    component of `F_total` on the x_b axis while `h<=0.001m` (models the
    launch rail holding the vehicle; ~8.7% lateral cross-coupling is left
    unconstrained by design). `LiftoffArm` latches the touchdown detector
    disabled until `h` first exceeds 1.0m, so the pad-phase residual dip
    (~-0.043m) doesn't falsely trigger it.
14. **Chaotic near touchdown**, confirmed twice: tiny numerical differences
    (solver tolerance, an unrelated latch addition) produce meter-scale
    differences in apogee/touchdown, while ascent/pad-phase numbers stay
    stable to ~1e-9. For the future sensitivity sweep, evaluate metrics
    during ascent, not near touchdown.
15. Full-flight baseline numbers from before the ascent-curve change are
    stale (ascent burn duration went from a manual 10s to the real
    ~21.5s). Current flight reaches touchdown at t~24.3s.
16. **Solver**: default `ode15s`/`RelTol=1e-6` was far tighter than needed
    (319k steps, 249s wall-clock for 24s of flight). Switched to `ode45`/
    `RelTol=1e-4` (25k steps, 30s, same final flight time, no accuracy
    loss observed). Currently running fixed-step `ode4`/0.001s instead
    (later preference, also verified fast, ~16s wall-clock).
17. **Workspace-pollution fragility**: `matl.m`/`lqr_gain_design.m` run as
    scripts sharing the base MATLAB workspace, so a stray leftover variable
    with a colliding name can silently break InitFcn (hit twice: a stale
    `rocket.Ixx`, then a `diag` variable shadowing the builtin `diag()`
    function). Fixed at the root: `matl.m` now runs `clearvars` as its
    first line, so InitFcn is self-healing regardless of prior workspace
    state.
18. **`Simulation Monitoring` subsystem** (root level, client request): a
    dedicated read-only review panel, separate from the per-block debug
    scopes scattered throughout the model (those stay in place). Contains
    one Scope per signal group: Position (`Xe`), Velocity (`Ve`), Attitude
    (`Euler`), Angular Rates (`Omega_be`), Thrust (per-motor), Forces
    (`F_total`), Moments (`M_total`). Fed by root-level `From` blocks
    reading the existing root Goto tags of the same names (one-level hop,
    per the Goto/From rule below) — no new signal taps were added to the
    plant, this only reuses what was already broadcast at root scope.

## Physics / math conventions
- Full variable-inertia Euler equation:
  `M = I*omega_dot + I_dot*omega + omega x (I*omega)`. `I_dot*omega` must be
  an explicit separate output from `MassInertiaModel`, not folded in.
- Quaternion attitude error (vector part), not Euler angle subtraction.
- No separate exhaust-velocity port; the thrust curve already encodes
  propulsive force.
- `x_b` is the vehicle's thrust axis (positive = up at the nominal
  near-vertical attitude). `h = -Xe(3)` (NED frame, `Xe(3)` positive-down).

## Style & engineering preferences
- Every editable parameter lives in `matl.m`, never hardcoded inside a
  MATLAB Function/Stateflow script (item 6 explains why — struct fields
  must come in as explicit input ports). Applies to the plant; the
  controller is off-limits for now (see Ownership split).
- Prefer native Simulink blocks over MATLAB Function blocks; reserve the
  latter for logic that genuinely needs it.
- No persistent variables in MATLAB Function blocks under continuous
  solvers.
- Modular architecture: gravity, thrust, drag, mass dynamics as separate
  blocks.
- All code comments in English. No em-dashes.
- Goto/From tags: never jump more than one subsystem level in a single tag.
  Use a real Outport at each boundary and reserve Goto/From for the final,
  adjacent hop only — a mis-routed tag can silently compile to a frozen
  constant sample time instead of erroring. Verify with
  `get_param(fromBlock,'CompiledSampleTime')` after adding any new pair.
- When a block's port order/count changes, rewire by signal name, never by
  raw port number.
- Build and test any new force/logic-summing block in isolation before
  wiring it into the live model.
- A new block isn't guaranteed to inherit a safe discrete sample time just
  because a sibling block does — check `CompiledSampleTime` explicitly if
  it uses persistent state.
