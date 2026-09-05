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
- **Descent guidance law (`descent_tilt_lqr`) isn't scaled for igniting
  far from the ground — real open item, not yet fixed**: `theta_throttle`
  saturates at its max (`gimbal_limit_hover_deg(2)` = 60°) for however
  long `h` stays large after ignition, because `delta_theta =
  -(K_H*h_err + K_V*v_vertical)` with `K_H=-0.2` blows past `theta_max`
  almost immediately once `h_err` is more than a few meters — the law was
  evidently tuned assuming ignition happens within a few tens of meters
  of the ground, not near apogee. At 60° tilt, `cos(60°)=0.5`, so half
  the descent thrust is wasted sideways (and actively adds lateral drift)
  for as long as saturation holds; useful braking only kicks in once `h`
  drops enough for `delta_theta` to come off the rail. This is *why*
  igniting earlier (higher up) doesn't keep improving touchdown vertical
  velocity past a point, and why it makes lateral drift worse the higher
  it's pushed (swept 15-76 m ignition altitude at the current ~76 m
  apogee: touchdown lateral drift went from ~5.6 m to ~27 m as vertical
  velocity improved from ~-36 to ~-16 m/s — a real Pareto tradeoff, not
  noise). Properly fixing this needs the guidance law itself reworked
  (e.g. clip/scale the `h_err` term, or gain-schedule `K_H` by altitude),
  not just a gain or timing tweak — out of scope for this pass since it's
  a `Controller` change beyond the gains already touched above; flagged
  for a follow-up.
- **Lateral drift fixed via LQR retuning, client-approved (touches
  `lqr_gain_design.m`, Umut's)**: was ~290 m by touchdown against a
  ~130-139 m apogee (root cause: pitch stayed near-vertical, but roll/yaw
  swung through 100°+ excursions, spinning the vehicle about its own
  thrust axis). Fixed empirically by de-weighting roll and raising
  pitch/yaw (angle AND rate) in `Q`: `diag([8,120,120,0.25,15,15])`,
  `R=diag([200,30,30])` (was `diag([20,20,20,0.5,0.5,0.5])` /
  `diag([200,200,200])`). Counterintuitively, *boosting* roll weight
  first (to compensate its weaker ~4x-shorter moment arm) made drift
  ~2x worse, not better — roll rotates about the thrust axis and doesn't
  tilt where thrust points, so fighting it hard just burns gimbal budget
  out of proportion via the shared saturation clamp in
  `tvc_controller_dcm`, starving pitch/yaw. Rate weight mattered as much
  as angle weight: with only angle weighted, pitch rate at ascent
  burnout was ~32 deg/s, which then free-tumbles the vehicle during the
  unpowered coast (no thrust = no gimbal authority to damp it) and wrecks
  attitude before descent-burn ignition even starts; raising rate weight
  to 15 brought that to ~12 deg/s. Current result: ~6-7 m lateral drift
  at touchdown (apogee ~76 m), down from ~290 m at the old ~139 m apogee.
  See `sweep_lqr.m`-style harness pattern in this file's git history if
  retuning further — hand-editing `lqr_gain_design.m`, resimulating, and
  reading `logsout`'s `XeRoot` for apogee/lateral drift was the whole
  method, no closed-form tuning.
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
- **Descent polish pass, client request (lower apogee, earlier descent
  ignition, hover-before-landing)**:
  - **Ascent burn shortened 10 s -> 7.6 s, then -> 6.7 s after the mass-bug
    fixes below** (still flat 8 N/motor, `thrust_data_ascent_clean.csv`) to
    bring apogee from ~130 m into the client's requested 70-80 m band.
    Found by direct simulation sweep (`t_burn_ascent` derives from the
    CSV's last timestamp, so this is just the CSV's second row): with the
    live-mass fix, apogee runs from ~40 m at 5 s up to ~84 m at 7 s,
    roughly but not exactly linear (lighter true mass late in the burn
    means more delta-v than the old buggy-gravity model assumed for the
    same burn time, hence a shorter burn now hits the same apogee target —
    see "Plant audit" below). 6.7 s currently lands at 76.15 m apogee.
  - **`Controller/Descent Throttle`'s hover equilibrium now targets 1 m
    above ground, not the ground itself (Umut's file, touched)**: added a
    `rocket.hover_altitude_m = 1` field and a new chart input
    (`hover_altitude_m`), and changed `delta_theta`'s height term from
    raw `h` to `h_err = h - hover_altitude_m`. One-line behavioral change,
    new Rocket-level `HoverAltitude` Constant block feeds the chart's new
    8th input port.
  - **`descent_ignition_altitude_m` stays at 40 m, re-swept after the
    mass-bug fixes — still a deliberate trade-off pick, not an arbitrary
    value**: re-swept 20-75 m with live mass in place (see the
    guidance-law bullet above for why it's a real Pareto tradeoff between
    lateral drift and touchdown vertical velocity, not just "more time is
    better"; the live-mass fix genuinely improved the *best available*
    touchdown vz — see "Plant audit" below). 40 m gives lateral drift
    ~14.8 m and touchdown vertical velocity ~-25.4 m/s — kept as the
    balanced point since neither metric dominates the client's asks; a
    different priority (accuracy vs. impact speed) would justify a
    different pick from this table, without needing another sweep:
    | ignition alt (m) | lateral drift @ touchdown (m) | touchdown vz (m/s) |
    |---|---|---|
    | 20 | 6.6 | -36.9 |
    | 30 | 9.3 | -32.1 |
    | **40 (current)** | **14.8** | **-25.4** |
    | 50 | 25.7 | -18.3 |
    | 60 | 30.2 | -9.0 |
    | 65 | 34.1 | **-6.4** |
    | 75 | 65.2 | -14.8 |
  - **Touchdown vertical velocity did not reach "close to zero" as the
    client asked, and can't with the current guidance law + ~8 N/motor**:
    best achieved (~-6.4 m/s at 65 m ignition, up from ~-15.8 m/s
    pre-mass-fix) is better but still a hard-ish landing, and only at the
    cost of ~34 m lateral drift. See the guidance-law-saturation bullet
    above — this is the same root cause. A real soft landing needs that
    law fixed (and/or more thrust margin); flagged, not resolved this
    pass.
  - None of this touched `t_burn_descent` (still 10 s) — burn duration was
    never the binding constraint in any of the sweep results above; the
    vehicle always hits the ground well before the descent motors would
    run out.
- **Plant audit (client-requested, before any controller architecture
  work) found and fixed 2 real physics bugs — not a controller problem**:
  - **Gravity used a static initial mass, not the live depleting one.**
    `Forces and Moments/Subsystem2/Subsystem1`'s gravity `Product` block
    multiplied `g_ned` by `Constant4 = rocket.m0_computed` (2.0 kg, fixed)
    instead of `MassInertiaModel`'s live `m` output — so weight force
    stayed at the full initial mass for the *entire* flight even as
    thrust/inertia correctly used the depleting mass elsewhere. By
    touchdown the true mass is ~1.5-1.55 kg (~65% burned on the current
    profile), a ~25-30% weight overestimate. Fixed by adding a `Mass`
    input port through `Subsystem2` -> `Subsystem1` (new port on each),
    wired from the `m` signal that was already computed and exposed at
    `Forces and Moments`'s own boundary but not connected further (it fed
    only its own dangling `m` outport). `Constant4` removed.
  - **Same bug, `Controller/Descent Throttle`'s hover-equilibrium calc**:
    `Constant6` fed `descent_tilt_lqr`'s `m_total` with
    `rocket.m0_computed` too, so `theta_hover = acos(m*g/T_max)` always
    assumed full initial mass, never the lighter actual descent-time
    mass. Fixed the same way: new `mass_live` outport on `Rocket` (port
    11), new root Goto tag `Mass`, new `Mass` inport on `Controller` (port
    7) and `MassLive` inport on `Descent Throttle` (port 5), replacing
    `Constant6`.
  - **Net effect of both fixes**: apogee for the same ascent burn duration
    increased (lighter real mass late in the burn = more net acceleration
    than the buggy model assumed), so the ascent burn needed re-shortening
    (7.6 s -> 6.7 s) to stay in the 70-80 m target band. The *correct*
    hover-equilibrium math also made touchdown vertical velocity
    genuinely improvable at higher ignition altitudes (best case improved
    from ~-15.8 to ~-6.4 m/s), confirming this was a real physics error
    with real consequences, not a cosmetic one.
  - **Also audited and confirmed correct, not bugs**: the TVC allocation
    matrix in `tvc_controller_dcm` is exactly the small-angle linearization
    of the actual nonlinear thrust-vector model in `rocket_forces_moments`
    (checked term-by-term); `AssembleRCG`'s dynamic CG tracking already
    used live `x_cg` correctly (the mass-bug pattern above did *not*
    recur there); the 6DOF block's `vre_flag=off` is correct (thrust is
    modeled as an explicit applied force, so mass-flow relative-velocity
    reaction would double-count it); `GroundReaction` and the `h` outport
    correctly use the unclamped altitude (the `Altitude Clamp` only feeds
    atmosphere/gravity models), matching what this file already claimed.
  - **Known, not fixed, flagged for later (out of scope for "just fix the
    bugs")**: the 6DOF block's initial attitude, `eul_0 = [0,
    deg2rad(85), 0]`, is hardcoded in the block mask (not sourced from
    `matl.m`, violating this project's own convention) and doesn't match
    `rocket.DCM_ref`'s ~89.9° — the sim starts ~4.9° off the controller's
    own reference for no physical reason. Also, mass/inertia depletion in
    `MassInertiaModel` is a function of absolute sim time
    (`t/rocket.mass_burn_duration`), not of whether the motors are
    actually firing — during any coast phase (a real feature of the
    current flight profile) it keeps "burning" propellant on the clock
    with no thrust. Both pre-date this pass; flagged, not touched.

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
