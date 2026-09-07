# Development Notes

Design rationale, known limitations, and coding conventions for this
project. For a guided explanation of what the model does, see
`MODEL_WALKTHROUGH.md`.

## Known limitations

- **CG at burnout**: `x_cg_burnout` (1.037 m) is derived assuming
  propellant burns from the motor pivot, not the true propellant
  centroid — a simplifying assumption, not a measurement.
- **Inertia model**: `I(t)` is diagonal-only, with no parallel-axis
  correction for the CG shift. `rocket.I_dry`/`I_prop` (in `matl.m`) are
  the only inertia numbers in the project; their own measurement
  provenance isn't documented in `matl.m`, so treat them as reasonable
  estimates rather than verified measurements.
- **Aerodynamic drag**: not modeled (`F_aero = 0`) — negligible at the
  current flight envelope; revisit if speeds/altitudes increase.
- **Chaotic near touchdown**: tiny numerical differences (solver
  tolerance, an unrelated model edit, RNG seed) can produce meter-scale
  differences in apogee/touchdown, while ascent stays stable. Evaluate
  any touchdown-affecting change across multiple noise seeds, not a
  single run — a "run once" or "compare 2 single-seed runs" result is
  not trustworthy this close to the ground. Sweep parameters in odd,
  not tight, steps, and check a candidate value's immediate neighbors
  rather than trusting a single "best" sample.
- **TVC allocation uses a static CG**: the controller's thrust-allocation
  matrix uses a fixed (t=0) moment arm, while the plant's actual
  force/moment mixing tracks the burning CG dynamically. Real asymmetry,
  not yet reconciled.
- **Descent thrust is a placeholder**: the descent CSV is wired into
  `Thrust` the same way as ascent, but it's currently the same idealized
  flat profile as ascent, not real descent motor test data.
- **Known open item**: a motor stronger than ~8-9 N/motor will likely
  destabilize the vehicle (drifts ~100°+ in attitude before crashing at
  high descent speed, observed at 30 N/motor in an ad hoc test) — the
  LQR gain `K` and TVC allocation were designed around the current
  thrust regime and would likely need their own redesign for a
  materially stronger motor, not just a data swap.
- **`matl.m`'s InitFcn requires the project root as MATLAB's working
  folder** at load/update time (see the README's Quick start). Not
  fixed at the root cause — a future improvement would resolve the path
  relative to the model file itself instead of relying on cwd.

## Design decisions

- **Suicide-burn descent ignition**: ignition is a real-time trigger,
  not a fixed altitude. `Thrust Status` ignites the descent motors the
  instant remaining altitude `h` drops to the distance needed to brake
  the current fall speed to zero using full (untilted) descent thrust,
  plus a safety margin (`stopping_distance =
  vertical_velocity^2 / (2*a_brake)`, `rocket.descent_ignition_margin_m`
  on top). This adapts automatically to whatever velocity disturbances
  actually occurred by a given altitude, instead of assuming a nominal
  trajectory — a perfectly-timed open-loop burn is fragile because real
  conditions (wind, lateral-guidance thrust diversion) eat into the
  vertical margin a fixed-altitude timing calc would have assumed.
- **Two-stage descent throttle**: `descent_tilt_lqr` commands full,
  untilted thrust (`theta_throttle=0`) until a one-way latch trips on
  `h <= hover_altitude_m` **or** `vertical_velocity >= 0`, then switches
  permanently to a fine velocity-feedback law targeting
  `v_target = -h / (remaining_fuel_time_s - reserve_time)`. The latch
  matters: an instantaneous (non-latched) check chatters right at the
  switch boundary, since full thrust decelerates the vehicle back
  through `v=0` and would otherwise immediately re-trigger full thrust,
  then fine control, then full thrust again — this was observed to
  destabilize attitude badly before the latch was added. `K_H` (the old
  height-error gain) is now dead — the fine-control law only uses `K_V`
  — kept as a field/input only to avoid removing the block's interface.
- **Lateral guidance**: closes the horizontal position/velocity loop
  nothing else in this architecture handles. During descent only,
  `lateral_guidance_dcm` biases `DCM_ref` off-vertical via a PD law on
  X/Y position and velocity error, clamped to
  `rocket.lateral_guidance_max_tilt_deg`. This is a real tradeoff, not
  free improvement — tilting to correct lateral error steals from the
  vertical thrust component and from the same gimbal-saturation budget
  the attitude controller shares across all axes, so drift and vertical
  speed can't both be driven to zero independently. Current gains hold
  worst-case vertical touchdown speed roughly at baseline while cutting
  worst-case lateral drift substantially.
- **TVC thrust-allocation Jacobian, linearized at the operating point,
  not at zero**: `tvc_controller_dcm`'s per-motor allocation matrix is
  built from each motor's current nominal (throttle-tilted) direction
  rather than the origin. A zero-point linearization is correct at
  `throttle_angle=0` (true during ascent) but silently misallocates
  thrust between the axial and lateral components once collective tilt
  gets large (up to 60° during descent) — the LQR attitude correction
  added on top of that tilt would otherwise be computed against the
  wrong local slope, stealing net vertical thrust from the throttle
  command without either loop seeing it happen.
- **LQR tuning**: roll is deliberately de-weighted relative to
  pitch/yaw — roll rotates the vehicle about its own thrust axis and
  doesn't tilt where thrust points, but its allocation moment arm is
  much shorter than pitch/yaw's, so fighting roll errors hard burns
  gimbal budget out of proportion via the shared saturation clamp in
  `tvc_controller_dcm`, starving pitch/yaw. Rate weights matter as much
  as angle weights: an under-damped rate response leaves residual
  pitch/yaw rate at ascent burnout, which then free-tumbles the vehicle
  during the unpowered coast phase (no thrust means no gimbal authority
  to damp it) and wrecks attitude before descent-burn ignition. The LQR
  design inertia is derived directly from the same dry/propellant
  breakdown the plant itself uses
  (`I_dry + I_prop*(1 - t_burn_ascent/mass_burn_duration)`), so it can't
  drift out of sync with the simulated body.
- **Single source of truth for parameters**: every plant/controller
  parameter lives in `matl.m` (`rocket.*`) and is fed into blocks as an
  explicit input, never hardcoded inside a MATLAB Function/Stateflow
  block — Stateflow can't resolve `rocket.*` fields internally. This
  includes `DCM_ref`, gimbal limits, and hover/lateral-guidance gains.
  The plant's initial attitude and the controller's reference attitude
  both derive from the same `rocket.launch_pitch_deg`, for the same
  reason — two independently hardcoded copies of a value that must
  agree is a standing invitation for them to quietly drift apart.
- **Live mass/CG throughout**: mass, inertia, and CG position all derive
  from one linear depletion fraction (`frac_burned`) each timestep, so
  they stay physically consistent with each other. Gravity and the
  descent hover-equilibrium calculation both use this live, depleting
  mass rather than the initial mass — using a stale initial mass
  anywhere overestimates weight by a large margin by touchdown, since
  a meaningful fraction of propellant has burned off by then.
- **Monitoring is display-only where it diverges from the real
  signals**: `Simulation Monitoring` scopes apply a few transforms that
  don't touch the underlying simulated signals — a Position/Velocity
  Z-axis sign flip (so "up" reads positive, matching the altitude
  convention), a causal Euler-angle unwrap (avoids a false ±360° jump
  when roll/yaw cross ±180°), and named Bus Creator signals so scope
  titles/legends stay meaningful. `Rocket`'s own outports, the root
  Goto/From tags, and `Controller`/`Animation` all keep the original
  convention unchanged.
- **`Animation` disabled by default**: the 3D viewer noticeably slows
  down iteration during testing/tuning, so it's commented out rather
  than deleted — still present in the model for whenever an actual
  visual check is wanted (`set_param('rocket_upwork/Animation',
  'Commented','off')`, or toggle from the canvas). Nothing else consumes
  its output, so disabling it doesn't affect simulation results.
- **Altitude clamp ahead of the atmosphere/gravity models**: guards
  against extreme negative-altitude values (from an unstable, crashing
  run) feeding the Aerospace Blockset atmosphere/gravity models values
  outside their valid range — a likely cause of severe sim slowdown in
  that case. Doesn't affect the real `h` used for touchdown detection,
  only this internal feed.
- **`Rocket` exposes one clean interface** (gimbal commands in; state
  and force/moment telemetry out), so swapping in a different control
  architecture or running a plant-only sweep doesn't mean untangling
  root-level wiring. `Thrust` lives at root level alongside `Rocket`,
  not inside it, since both `Controller` and `Rocket` need its
  per-motor thrust output.

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
  inside a MATLAB Function/Stateflow block — any block needing one takes
  it as an explicit input port.
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
