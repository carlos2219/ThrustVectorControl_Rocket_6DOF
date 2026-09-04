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
- **Descent is a placeholder**: the descent CSV is wired into `Thrust`
  the same way as ascent, but it's currently the same idealized flat
  profile as ascent (see below), not real descent motor test data.
  Current focus is ascent only.
- **Large lateral drift, root cause identified, fix not yet applied**:
  ~290 m lateral drift by touchdown against a ~130-139 m apogee. Pitch
  stays near-vertical throughout (looks "upright"), but roll/yaw swing
  through 100°+ excursions — the vehicle spins about its own thrust axis
  and that's what drags it sideways. Root cause: the LQR weights in
  `lqr_gain_design.m` (Umut's) are symmetric across all 3 attitude axes
  (`Q = diag([20,20,20,...])`), but roll has a much shorter moment arm
  (`r_arm` ≈ 0.025 m vs. `engine_pivot_x_from_cg` ≈ 0.1 m for pitch/yaw)
  and a much smaller inertia (`Ixx_burn` = 0.018 vs. `Iyy_burn`/`Izz_burn`
  = 0.338), an asymmetry the current design doesn't account for. Retuning
  `lqr_gain_design.m` is next, deliberately deferred to a separate pass —
  do not touch it as a side effect of unrelated work.
- **`matl.m`'s InitFcn requires the project root as MATLAB's working
  folder** at load/update time (see the README's Getting Started). Not
  fixed at the root cause — a future improvement would resolve the path
  relative to the model file itself instead of relying on cwd.

## Recent additions

- **`Simulation Monitoring` subsystem** (root level): one place with a
  scope per signal group — Position, Velocity, Attitude, Angular Rates,
  Thrust, Forces, Moments — for reviewing a run without hunting through
  the model. Per-block debug scopes elsewhere are unchanged.
- **Model restructured into 5 top-level subsystems**: `Rocket` (plant),
  `Controller` (LQR/DCM TVC + descent throttle), `Animation`,
  `Simulation Monitoring`, and `Flight Termination`. `Rocket` exposes one
  clean interface (`alpha_cmd`/`beta_cmd` in; state + force/moment
  telemetry out), so swapping in a different control architecture or
  running a plant-only sweep no longer means untangling root-level wiring.
- **`DCM_ref` and the gimbal limits are now genuinely single-sourced from
  `matl.m`**: the TVC controller reads `rocket.DCM_ref` and
  `rocket.gimbal_limit_ascent_deg` (previously hardcoded literals inside
  the MATLAB Function/Constant blocks); the descent throttle reads
  `rocket.gimbal_limit_hover_deg(2)`. No plant/controller parameter should
  be hardcoded inside a block — if you find one, it belongs in `matl.m`.
- **`matl.m` cleaned up**: reorganized by function (aero/geometry, mass,
  inertia, motor geometry, attitude reference, thrust/timing, gimbal
  limits, servo), and dead fields with zero consumers removed
  (`diameter`, `m_casing_each`, `burn_rate_each`, `T_total_nominal`,
  `rocket.m`, `rocket.I_burn`, `rocket.lqr.DCM_ref`).
- **Thrust is now a single root-level `Thrust` subsystem, sourced directly
  from the CSVs — the artificial/real Manual Switches are gone.** Both
  `thrust_data_ascent_clean.csv` and `thrust_data_descent_clean.csv` now
  have 4 columns: `time_seconds, thrust_m1_N, thrust_m2_N, thrust_m3_N`
  (per-motor columns are currently identical, duplicated from the old
  single-sensor curve, until real per-motor test data exists). `matl.m`
  loads each into a 3xN array (`rocket.ascent_thrust_curve_N`,
  `rocket.descent_thrust_curve_N`); the actual interpolation happens in
  six native `1-D Lookup Table` blocks inside `Thrust` (3 per motor per
  phase, `ExtrapMethod=Clip` to replicate the old hold-last-value
  behavior) — no more `interp1` in a MATLAB Function.
  `T_nominal`, `thrust_pert`, and `ignition_delay` were removed from
  `matl.m` (no remaining consumers).
  - `Thrust` moved out of `Rocket` entirely: it takes `height`/`Velocity`
    feedback from the root Goto/From bus (same signals every other
    subsystem reads) and outputs the per-motor `T_per_engine` 3-vector and
    `phase` onto the existing root `Thrust`/`phase` Goto tags. `Rocket`
    lost its `Thrust`/`faz` outports and gained a `T_per_engine` inport
    (port 3) feeding straight into `Forces and Moments`; it's now a pure
    force/moment integrator with no thrust-generation logic of its own.
  - The phase-detection state machine (ascent burn -> coast -> descent
    burn) is unchanged logically, just relocated and trimmed (its old
    unused flat-`T_per_engine` output was dead code — nothing consumed it
    — so the rebuilt chart only outputs `phase` and
    `remaining_fuel_time_s`). **It's a MATLAB Function block with
    `persistent` state, which requires an explicit discrete
    `SystemSampleTime` (set to `0.001`, matching the fixed-step solver) —
    Simulink errors at simulation start if this is left at `-1`
    (inherited/continuous) with persistent variables in play.**
  - **Controller/Descent Throttle (Umut's, off-limits territory) was
    touched**: its hover-throttle equilibrium calc (`descent_tilt_lqr`)
    used to read the flat `rocket.T_nominal` scaled by `n_engines`; it now
    takes the live `T_per_engine` vector (already available at
    `Controller`'s boundary) and uses `sum(T_per_engine)` instead. Flag
    this to Umut — the math is equivalent when the vector is flat, but it
    now legitimately varies over the descent burn instead of being a
    constant assumption.
  - Deleting the two internal Rocket-level Goto/From pairs that fed the
    old in-`Rocket` `Thrust Subsystem` (`Ve`/`Xe`, tags reused by
    `From5`/`From6` for `Forces and Moments`' own `Ve`/`Xe` inputs) is an
    easy way to silently zero out `h` and everything downstream of it —
    Simulink does not error on the mismatched Goto/From tags at compile
    time, it just holds the last (zero) value. If `h` ever again reads as
    suspiciously flat/zero for a whole run, check for exactly this.
- **`Altitude Clamp`** (`Saturate`, `[0, inf]`) sits ahead of the `ISA
  Atmosphere Model` and `WGS84 Gravity Model` in the aero subsystem. Added
  while chasing an extreme sim slowdown during an unstable high-thrust
  crash test: once the vehicle punches through the ground with a large
  negative altitude, those Aerospace Blockset models are fed values
  outside their valid range, which is the likely cause of the slowdown.
  Clamping the altitude feed (not the real `h` used for touchdown
  detection, only this branch) keeps atmosphere/gravity well-behaved
  regardless of how badly a given run crashes.
- **Known open item, carried forward**: a motor stronger than ~8-9 N/motor
  will likely destabilize the vehicle (drifts ~100°+ in attitude before
  crashing at high descent speed, observed at 30 N/motor in an earlier ad
  hoc test) — the LQR gain `K` and TVC allocation were designed around
  this regime and likely need their own redesign for a materially
  stronger motor, not just a data swap. Also see "Descent is a
  placeholder" above, still true.
- **Thrust CSVs replaced with an idealized flat profile, client request**:
  `thrust_data_ascent_clean.csv` and `thrust_data_descent_clean.csv` are
  now just 2 rows each — `[0, 8, 8, 8]` and `[10, 8, 8, 8]` — i.e. a flat
  8 N/motor for 10 s, 0 N during coast, 8 N/motor for 10 s on descent.
  Since `t_burn_ascent` derives from the CSV's own last timestamp, this
  changed it from ~21.5 s to exactly 10 s automatically, no `matl.m` edit
  needed. The previous real per-motor static-test curves are archived at
  `thrust_data_archive/*_real_static_test.csv` (not read by anything,
  reference only). New apogee ~130 m (was ~139 m with the real curve).
  Also added a legend + clearer title to the `Simulation Monitoring`
  `Thrust (per motor)` scope, so the combined ascent+coast+descent profile
  reads unambiguously as one continuous signal (it always was one signal
  end-to-end after the `Thrust` subsystem rework above — this was a
  labeling fix, not a wiring fix).
- **Per-motor gimbal telemetry + Z-up plots, client request (motor
  orientation/rate plots, Z sign convention)**:
  - `Rocket` gained two new outports, `alpha_real`/`beta_real` (ports 9,
    10) — the achieved per-motor gimbal angles, tapped from the existing
    `Servo Actuator`/`Servo Actuator1` outputs (post servo dynamics, not
    the raw TVC-controller command). New root Goto tags `GimbalAlpha`/
    `GimbalBeta` carry them into `Simulation Monitoring`.
  - Four new scopes in `Simulation Monitoring`: `Gimbal Angle - Alpha/Beta
    (deg)` and `Gimbal Rate - Alpha/Beta (deg per s)`. Angles are
    `alpha_real`/`beta_real` converted rad->deg; rates are a `Discrete
    Derivative` block on the deg signal (inherits the fixed-step solver
    rate) — deliberately **not** implemented by exposing the servo's
    internal rate state (would mean changing the `Servo Actuator`
    State-Space `C` matrix, i.e. editing Umut's file; the discrete
    derivative gets the same information without touching it). Expect the
    derivative to read a few % above the `servo.max_rate_dps` = 90 limit
    at the sharpest transitions (~98 seen in testing) — that's
    discretization overshoot from differentiating a fixed-step signal, not
    an actual rate-limit violation (the real `Rate Limiter` blocks enforce
    90 deg/s upstream, before the servo dynamics smooth it further).
  - `Position (Xe)` and `Velocity (Ve)` scopes in `Simulation Monitoring`
    now flip the sign of their Z component only (`Demux` -> `Gain(-1)` on
    Z -> `Mux` back to 3-vector) so the plots read positive going up,
    matching the existing `h` (altitude) convention. This is a **display-
    only** change scoped to these two scopes — `Rocket`'s own `Xe`/`Ve`
    outports, the `Position`/`Velocity` root Goto tags, `Controller`, and
    `Animation` all keep the original NED (Z-down) convention unchanged.

- **`Simulation Monitoring` scope labeling convention, client request**:
  every vector scope (`Attitude (Euler)`, `Angular Rates`, `Forces`,
  `Moments`, `Position (Xe)`, `Velocity (Ve)`, `Thrust (per motor)`, the 4
  gimbal scopes) now feeds through a `Demux` -> `Bus Creator` instead of a
  plain `Mux`. The scope `Title` stays the default `%<SignalLabel>`
  template — it's the **signal name on the Bus Creator's output line**
  that supplies the title text (e.g. named `Attitude`), and the **names
  on the Bus Creator's input lines** that supply the per-line legend
  entries (e.g. `Roll`/`Pitch`/`Yaw`) — Simulink reads both from the line
  `Name` property, not the port. `YLabel` carries the units instead of the
  title (`degrees`, `m`, `N`, etc.). `Attitude (Euler)` and `Angular
  Rates` also gained a `180/pi` `Gain` — previously plotted in radians
  despite looking like degrees, now genuinely degrees. The root-level
  `Euler`/`Omega_be`/`Xe`/`Ve`/etc. signals themselves are untouched
  (still radians/NED where applicable) — all of this is scoped to the
  monitoring plots only. The 6-panel `Scope` block (dashboard duplicating
  the individual scopes) now sources from the same named bus signals
  instead of the raw Inports, so its titles/legends stay in sync
  automatically — but it has no working per-axis `YLabel` (tested: it's a
  single block-wide string, useless with 6 different units on one block),
  so that one has no Y-axis units, unlike every dedicated scope.

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
