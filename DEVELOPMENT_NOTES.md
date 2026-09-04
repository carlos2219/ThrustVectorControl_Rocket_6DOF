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
- **Descent is a placeholder**: descent motors run on a flat nominal
  thrust value; the descent thrust curve CSV is loaded but not wired in.
  Current focus is ascent only.
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
- **Artificial test-thrust Manual Switches** in `Rocket/Thrust Subsystem`,
  now three, not two — **must be flipped together** (see the note block
  next to them in the model):
  - "Ascent Thrust Source": real curve (`rocket.ascent_thrust_curve_N`) vs.
    flat artificial (`rocket.test_ascent_curve_N`).
  - "Ascent Burn Duration Source" (new): how long phase 1 lasts —
    `rocket.t_burn_ascent` (real, ~21.5 s, derived from the curve's last
    timestamp) vs. `rocket.test_ascent_thrust_duration_s` (new field, 10 s,
    client-specified). Before this switch existed, `t_burn_ascent` fed
    `Thrust Status` unconditionally, so the artificial-thrust test always
    ran for ~21.5 s regardless of the other switch — this is what caused
    the 632 m vs. ~280 m apogee discrepancy the client reported (excess
    impulse from burning ~11.5 s longer than intended, not a double-count
    or double-gravity bug). Fixed by adding this switch; real-mode apogee
    unchanged (138.8 m, verified), artificial-mode apogee dropped from
    631.6 m to 130.0 m once the duration matches the intended 10 s.
  - "Descent Thrust Source": real/nominal (`rocket.T_nominal`) vs. flat
    artificial (`rocket.test_descent_thrust_N`).

  `CurrentSetting = '1'` selects real motor data on all three, `'0'`
  selects the artificial test values. Built for quick what-if checks (e.g.
  "would a stronger motor reach a reasonable apogee?") without editing the
  CSVs. **Currently all three switches are left at `'0'`** (artificial
  thrust, 8 N per motor, 10 s) while this test is ongoing; flip all three
  to `'1'` to go back to the real ascent/descent curves for normal runs.

  **Field map** (ascent and descent are intentionally different
  mechanisms — ascent has a real motor curve, descent doesn't yet, so it
  runs on a flat scalar):

  | `matl.m` field | Consumer(s) | Role |
  |---|---|---|
  | `ascent_thrust_curve_t/N` | `PerMotorThrust` | real ascent curve |
  | `test_ascent_curve_N` | `PerMotorThrust` | flat artificial ascent |
  | `t_burn_ascent` | `Thrust Status` | real ascent burn duration |
  | `test_ascent_thrust_duration_s` | `Thrust Status` | artificial ascent burn duration |
  | `T_nominal` | `PerMotorThrust` **and** `Controller/Descent Throttle` (Umut's hover-throttle calc) | descent thrust — no real curve yet, and shared with the controller, so don't rename/remove without checking that side |
  | `test_descent_thrust_N` | `PerMotorThrust` | flat artificial descent, independent of the controller's assumption |
  | `t_burn_descent` | `Thrust Status` | descent burn duration (already a single independent parameter, unaffected by this pass) |
  | `descent_thrust_curve_t/N` | none yet | loaded, reserved for a real descent curve (follow-up work) |
- **TVC allocation matrix now uses live per-motor thrust**: `tvc_controller_dcm`
  took a fixed `nominal_thrust` scalar (`rocket.T_nominal`) to convert
  commanded moment into gimbal angles — harmless with the real curve
  (peaks at ~9.2 N, close to the 8 N assumption) but it let the artificial
  30 N test thrust destabilize the vehicle immediately (the allocation was
  scaled ~4x off from reality). Fixed by feeding `T_per_engine` (Rocket's
  live `Thrust` output, routed through a new `Controller` input) into the
  allocation calc instead. Real-mode flight is very slightly different
  now (138.8 m apogee vs. 139.1 m before) since the allocation legitimately
  tracks the real curve instead of a flat assumption — expected, not a bug.
- **`Altitude Clamp`** (`Saturate`, `[0, inf]`) added ahead of the `ISA
  Atmosphere Model` and `WGS84 Gravity Model` in the aero subsystem. Found
  while chasing the extreme sim slowdown during the unstable 30 N crash
  test below: once the vehicle punches through the ground with a large
  negative altitude, those Aerospace Blockset models are being fed values
  outside their valid range, which is the likely cause of the slowdown.
  Clamping the altitude feed (not the real `h` used for touchdown
  detection, only this branch) keeps atmosphere/gravity well-behaved
  regardless of how badly a given run crashes.
- **Known open item from this pass**: with the artificial 30 N/motor
  thrust, the vehicle no longer tumbles but still drifts significantly in
  attitude (Euler swings ~100°+) before crashing at high descent speed —
  apogee ~344 m at t=7.1s. The LQR gain `K` and allocation were designed
  around the real ~8-9 N regime; a much stronger motor likely needs its
  own gain redesign, not just the allocation-scale fix above. Left for a
  follow-up pass — see also "Descent is a placeholder" above, which still
  applies (no real hover-control work done this round).

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
