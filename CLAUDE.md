# TVC Rocket 6DOF Simulation — Project Context

## Project overview
Freelance MATLAB/Simulink project (client: Kruthick Jothimani). Deliverable: a 6DOF
flight simulation modeling a three-motor TVC rocket burning KNSB propellant.
Design philosophy from client: simplicity, avoid over-engineering. Milestone-based,
open-source intent. Physical rocket parameters are not final (client's rocket is
still under construction).

**Client's stated end goal (2026-08-26):** a single, abstracted, elegant
plant block representing the rocket, decoupled enough from the controller
that different control methodologies can be probed against it. Current
work polishes the plant first (mass/inertia/thrust/CG realism); the
controller (Umut's TVC/servo subsystems) is explicitly untouched until the
plant is considered functional, at which point the client intends to
revisit controller architecture separately.

## Repository layout
All active files now live flat at the PROJECT root - the old `UMUT/`
subfolder was removed this session (client preference for a single project
folder). See the cwd fragility note below for what this changes.
- `rocket_upwork.slx`: THE canonical master model, single source of truth.
  Originally Umut's deliverable (mature Servo Actuator subsystem, TVC DCM
  Controller with LQR/DCM-based attitude error, "vee map"); as of the Phase A
  pivot this is where all integration work happens directly. No longer
  read-only, no longer mirrored into a separate SIM_model.
- `matl.m`: the real InitFcn source for `rocket_upwork.slx` (model InitFcn
  callback is the bare string `matl;`). Builds the `rocket` struct the model
  needs: rocket.DCM_ref, rocket.cg, rocket.cp, rocket.m, rocket.diameter,
  rocket.m_dry, rocket.n_engines, propellant masses, burn rate,
  rocket.x_cg_initial/rocket.x_cg_burnout, etc. Calls `lqr_gain_design;` the
  same bare way. Both resolve only if MATLAB's cwd is the PROJECT root at
  load/update time, see the cwd fragility note below. `matl.m` is now the
  single source of truth for every editable plant parameter, including
  MassInertiaModel's mass/inertia/burn-duration literals and Thrust Status's
  descent ignition altitude, both previously hardcoded inside their MATLAB
  Function block scripts (see Known gaps item 7).
- `lqr_gain_design.m`: computes rocket.lqr.K (LQR gain matrix), invoked from
  `matl.m`. Bug fixed this session: it was silently reading `rocket.Ixx`/
  `Iyy`/`Izz` (unsuffixed), fields `matl.m` never actually defines (only
  `Ixx_burn`/`Iyy_burn`/`Izz_burn` exist) - it only worked because a stale
  `rocket.Ixx` from an earlier session was still sitting in the base MATLAB
  workspace. A fresh `clear rocket; matl;` reproduced the failure
  (`Unrecognized field name "Ixx"`) before the fix confirmed it. Now
  correctly references `Ixx_burn`/`Iyy_burn`/`Izz_burn`.
- `initFcn.m` (root): legacy, from the retired SIM_model era. Not executed by
  `rocket_upwork.slx`. Kept for reference only, not an active init path.
- `NOTES.md` (root): working notes, currently just the cwd fragility writeup.
- `ARCHIVE/`: retired files, the whole pre-pivot SIM_model lineage
  (`SIM_model.mdl`, `SIM_model_backup.slx`, `SIM_model_model_befMERGE.slx`,
  `SIM_model.slxc`), the redundant root `lqr_gain_design.m` (confirmed
  byte-identical to the active `lqr_gain_design.m` at the time it was
  archived), the orphaned `MassInertiaModel.m` (not Umut's work, a stray
  copy of Carlos's own file, see `ARCHIVE/README.txt`),
  `rocket_upwork.slx.original` (historical pristine reference, no longer the
  active source of truth, diverged from the active file before this session
  even started), `rocket_upwork.slx.r2024b` (Simulink's own auto-backup
  from a format upgrade on save), and `rocket_upwork_OLDCHECK.slxc` (a
  stray backup-looking build cache found alongside the active files during
  the UMUT-folder consolidation; moved here rather than deleted since its
  origin is unclear).
- `thrust_data.csv`: raw static motor test data (scale reading, kgf,
  0.5s samples), extracted from a client video of a motor firing test.
  Includes pre-ignition dead time (t=0 to ~4s, exact zeros) and a
  post-burnout tare-drift artifact (t>=26.5s, negative readings, physically
  impossible as thrust). Kept untouched for traceability, no longer read
  directly by `matl.m`.
- `thrust_data_ascent_clean.csv` (renamed from `thrust_data_clean.csv` this
  session): user-cleaned copy of the above, pre-ignition dead time and the
  tare-drift artifact trimmed off, time re-zeroed so t=0 is real motor
  ignition. `matl.m` now loads this file IN FULL for
  `rocket.ascent_thrust_curve_t/N` - no longer truncated to a manually
  chosen duration, see Known gaps item 12. Still RAW scale-reading data, not
  corrected for motor mass loss during the burn.
- `thrust_data_descent_clean.csv` (new this session): currently just a
  duplicate of the ascent file's data, a placeholder the client can
  overwrite directly with real descent motor static-test data (same column
  format). `matl.m` loads it into `rocket.descent_thrust_curve_t/N`, but
  this is NOT yet wired into `PerMotorThrust` - descent still runs on the
  flat `rocket.T_nominal` placeholder, see Known gaps item 12.

## Ownership split
- Carlos: master Simulink file, full implementation/integration layer, Simscape/HIL,
  STM32 firmware, PCB, CAD. Practical end-to-end ownership.
- Umut: mathematical modeling, controller architecture (MSc Aircraft Engineering,
  incoming PhD Flight System Dynamics). Control algorithm dev is parallel/comparative
  between both.

## Legacy: SIM_model / rocket_6dof (archived, no longer active)
Carlos's original master plant model before the Phase A pivot. Retired to
`ARCHIVE/SIM_model.mdl` once its `MassInertiaModel(t)` block was ported into
`rocket_upwork.slx` (see "x_cg(t) integration" below). Kept only as historical
reference for pieces not yet ported:
- `Custom Variable Mass 6DOF (Quaternion)`: core plant. Quaternion to avoid gimbal
  lock at 90° pitch (nominal vertical-ascent condition). `rocket_upwork.slx`
  now uses this exact block (migrated from a legacy masked block, see known
  gaps item 1) with `mtype=Custom Variable` and quaternion state - reconciled.
- `ThrustMixer`: MATLAB Function block, net body-frame force/moment from three
  gimbaled motors via `R_x(gamma) * R_z(beta) * R_y(alpha)`. Superseded by
  `rocket_upwork.slx`'s own `rocket_forces_moments`, which already existed
  there independently with an equivalent alpha/beta motor-mixing approach,
  not ported, not needed.
- Gravity: constant rotated into body frame via DCM.
- `DragForce`: `F_drag = -0.5 * rho * |v|^2 * Cd * A_ref * v_hat`, in a
  `Forces and Moments` subsystem. Not yet present in `rocket_upwork.slx`.
- Parameters as struct fields in Constant block Value fields directly
  (`rocket.Cd`, etc.), avoid `assignin`/flat workspace variables. Still the
  right pattern going forward.

## rocket_upwork.slx is the canonical master model (read-only rule retired)
As of the Phase A pivot, `UMUT/rocket_upwork.slx` is the single actively-edited
master model, the old "never save_system on this file" rule from the
SIM_model era no longer applies. All integration work happens directly in
`rocket_upwork.slx`. Editing convention established during the x_cg(t) work:
new logic goes into new MATLAB Function blocks placed inside the same
subsystem as the related existing block (e.g. the new `MassInertiaModel` and
`AssembleRCG` blocks live alongside `rocket_forces_moments` inside
`Forces and Moments/Forces and Moments/`), not loose at the model root.

`UMUT/rocket_upwork.slx.original` is kept in `ARCHIVE/` purely as a historical
snapshot, it was already found to have diverged from the active file before
Phase A started (a prior session saved a cosmetic subsystem-grouping edit to
it, in violation of the old read-only rule), so it is not a reliable "pristine
Umut baseline" any more, just a reference point.

## Known gaps (confirmed live via MCP, keep this list updated)
1. **6DOF block identity: migrated to the modern block, DONE, verified, saved.**
   Formerly the obsolete masked block described below (kept for history: its
   `ReferenceBlock` was `aerolibobsolete/6DOF (Euler Angles)`, a single generic
   masked subsystem from the obsolete library covering both representations
   (`rep` mask param: EulerAngles/Quaternion) and all three mass modes
   (`mtype` mask param: Fixed/SimpleVariable/CustomVariable) in one block; it
   had no CG/reference-point port, trusting `Moments` and `I` to be about the
   same point, requiring `I` about the instantaneous CG by construction).
   Migrated to `aerolib6dof2/Custom Variable Mass 6DOF (Quaternion)`, the
   clean non-legacy block from the current Aerospace Blockset. Port diff:
   dropped `dm/dt` and `V_re` inputs (`vre_flag` left `off`, matches the old
   block's effective behavior of feeding a zero `V_re`), dropped `dwb/dt` and
   `A_bb` outputs (both were unused on the old block, fed only Terminators),
   added `q` (quaternion) output, currently unconnected, available for future
   use. **Gotcha for any future edit touching this block: `q` is inserted at
   output position 4, shifting `DCMbe`/`Vb`/`omega_be` down one port index
   versus the old block. Always wire and verify by signal name, never by raw
   port number** (see Style & engineering preferences). Init params (`xme_0`,
   `Vm_0`, `eul_0`, `pm_0`, `mass_0`, `mass_e`, `mass_f`, `inertia`,
   `inertia_e`, `inertia_f`) carried over 1:1. Three old params have no
   equivalent on the new block: `rep` (representation is now fixed by block
   choice, not a parameter), `k_quat` (quaternion normalization gain, removed
   by MathWorks in R2025a), `abi_flag` (no replacement found; was already
   `off`/inactive on the old block, so low-risk). Verified equivalent to the
   old block via a fixed-step solver isolation test (`ode4`, eliminates
   step-selection sensitivity): max diff 2.0e-11 m position, 1.6e-10 m/s
   velocity across the full flight, essentially machine precision. The
   variable-step solver showed larger apparent divergence near touchdown
   (~0.3 m / 20 m/s) but this was proven to be pure step-selection
   sensitivity near the chaotic impact event, not a real old-vs-new
   difference, by re-running the SAME new block at two different `RelTol`
   values and seeing comparable divergence. See item 14 for the broader
   chaotic-sensitivity finding this surfaced.
2. **DCM_reference hardcode mismatch in rocket_upwork**: informational only,
   not re-verified this session. Noted for Umut's awareness if relevant.
3. **x_cg(t) integration, DONE (Phase A, verified, saved).** Axial motor
   moment-arm now tracks propellant burn:
   `MassInertiaModel(t)` (4 outputs: `m, dIdt, I, x_cg`) -> `AssembleRCG` ->
   dynamic `r_cg` (row 1 axial, rows 2-3 static lateral) -> `rocket_forces_moments`.
   All three blocks live in `Forces and Moments/Forces and Moments/`, same
   subsystem as `rocket_forces_moments`. `AssembleRCG` formula:
   `r_cg(1,i) = x_cg - engine_pivot_from_nose` (rows 2-3 unchanged:
   `r_arm*cos/sin(azimuth_deg(i))`). Verified numerically: at t=0
   (`x_cg=rocket.x_cg_initial=1.09`) row 1 reproduces the model's prior static
   value exactly (-0.10 m, sign and magnitude); at burnout
   (`x_cg=rocket.x_cg_burnout=1.05`) row 1 = -0.14 m. Update Diagram compiles
   clean, model_check shows zero new errors/warnings versus the
   pre-integration baseline. `m`, `dIdt`, `I` outputs of `MassInertiaModel`
   are intentionally left unconnected, that is Phase B, not done.
4. **`r_cg_t` removed, does not exist.** The old `[m, dIdt, I, r_cg_t]`
   signature (with `r_cg_t` declared but never assigned) is gone, dropped
   during the x_cg port because Stateflow's eML dialect hard-requires every
   declared output be assigned somewhere in the function body, independent of
   downstream wiring; leaving it unassigned made `Update Diagram` fail outright
   (`Stateflow:translate:SFcnErrorStatus`). `AssembleRCG` is the one live
   mechanism for the dynamic moment arm now, do not reintroduce r_cg_t as a
   second, competing path for the same concept.
5. **`rocket.x_cg_initial` / `rocket.x_cg_burnout`: real client-measured data
   now in place, DONE, verified, saved.** No longer placeholders. Client-
   measured values now in `UMUT/matl.m`: `x_cg_full = 1.090 m` from nose tip
   (measured by balancing, fully fueled) and `x_motor_pivot = 1.190 m` from
   nose tip (measured). `rocket.x_cg_initial = 1.090 m` uses `x_cg_full`
   directly. `rocket.x_cg_burnout = 1.0373 m` is DERIVED, not measured:
   `x_cg_burnout = (m_total*x_cg_full - m_prop_total*x_motor_pivot) / m_dry`,
   assuming propellant mass acts as if concentrated at `x_motor_pivot`, a
   simplifying assumption for M1, not the true propellant centroid within the
   150 mm motor tube. Verified this reproduces `r_cg(0) = -0.100 m` exactly
   through `AssembleRCG`, same check as Phase A. See item 11 for the
   companion `m_dry` reconciliation this depends on.
6. **eML/Stateflow chart scripts cannot resolve `rocket.*` struct fields
   internally**, confirmed empirically (`Stateflow:cdr:ErrorsParsingEmlFcn`)
   while building `AssembleRCG`. Any MATLAB Function block in this model needs
   struct-derived values (`rocket.engine_pivot_from_nose`, `rocket.r_arm`,
   `rocket.azimuth_deg`, etc.) passed in as explicit input ports fed by
   Constant blocks with `Value = rocket.<field>`, never referenced directly
   inside the chart body. This is why `rocket_forces_moments` takes `r_cg` as
   an input rather than reading it, and why `AssembleRCG` does the same for
   its three geometry parameters.
7. **Duplicate parameter sets: fully resolved for MassInertiaModel, DONE,
   verified, saved.** `MassInertiaModel` no longer hardcodes ANY parameter
   locally. `m_dry`, `m_prop`, `t_burn`, `I_dry`, `I_prop` are now explicit
   input ports fed by Constant blocks (`Value = rocket.m_dry`,
   `rocket.m_prop_total`, `rocket.mass_burn_duration`, `rocket.I_dry`,
   `rocket.I_prop`) inside `Forces and Moments/Forces and Moments`, same
   single-hop pattern as `XcgInitial`/`XcgBurnout`. `rocket.I_dry`/
   `rocket.I_prop` already existed in `matl.m` before this session but were
   silently unused - `MassInertiaModel` had its own byte-identical hardcoded
   copies instead (`Ixx_dry=0.002` etc.), exactly the kind of duplication
   flagged for review; now `matl.m`'s fields are the only copy, passed in as
   3x3 matrices (`I = I_dry + I_prop*(1-frac_burned)` works directly on the
   matrices since both are diagonal by construction, no need to unpack to
   scalars). `rocket.m_prop_total` is a new field
   (`n_engines*(m_prop_ascent_each+m_prop_descent_each) = 0.690`, matches
   the old hardcoded value exactly). `t_burn` renamed to
   `rocket.mass_burn_duration = 20` and moved to `matl.m` - INTENTIONALLY
   still independent from `rocket.t_burn_ascent`/`t_burn_descent` (this
   model's mass depletion is one continuous linear ramp from t=0, not two
   separate phase burns), flagged to the user rather than silently merged.
   Verified via `Update Diagram` (clean) and a 40s `sim()` from a fresh
   `clear rocket; matl;` state (no stale-workspace masking possible).
8. **`MassInertiaModel`'s `I` is diagonal-only and NOT coupled to `x_cg(t)`.**
   `I = I_dry + I_prop*(1-frac_burned)` is an independent linear blend of
   `rocket.I_dry`/`rocket.I_prop` (now fed in from `matl.m`, see item 7 -
   this item is about the modeling simplification, not the parameter
   source), with no parallel-axis correction tied to the CG shift `x_cg(t)`
   tracks. Since the 6DOF block requires `I` about the instantaneous CG (item
   1), this is a known, deliberate simplification, not a rigorous model,
   accepted for M1 per the client's simplicity philosophy - explicitly
   reconfirmed acceptable by the client this session, on the condition that
   every parameter still lives in `matl.m`, not hardcoded inside a function
   (satisfied by item 7). Revisit if a sensitivity study shows high
   sensitivity to inertia. Documented in a comment inside `MassInertiaModel`'s
   script itself, not just here.
   **Measurement status of `rocket.Iyy`/`rocket.Izz`/`rocket.Ixx` in
   `matl.m`** (the LQR-facing inertia, separate from `MassInertiaModel`'s own
   local `Ixx_dry`/`Iyy_dry`/`Izz_dry` literals above): `Iyy = Izz = 0.338
   kg*m^2` are measured (bifilar pendulum). `Ixx = 0.018 kg*m^2` REMAINS
   Umut's placeholder guess, not measured, still open, pending a bifilar
   pendulum retest once the new carbon rods arrive.
9. **Phase B: variable mass/inertia wired to the 6DOF block, DONE, verified,
   saved.** `mtype` flipped from `Fixed` to
   `Custom Variable` on `blk_2`. Outports unchanged at the time (still 8,
   same order) - superseded by the item 1 migration, which changed the
   6DOF block's outport count/order entirely (7 outputs, `q` inserted at
   position 4). This item is kept as history of the Phase B mass-wiring
   work itself, which the migration did not undo.
   `MassInertiaModel` (`blk_431`, in `Forces and Moments/Forces and Moments/`)
   is 5 outputs: `m, dIdt, I, x_cg, m_dot` (m_dot added this round, analytic:
   `-m_prop/t_burn` while burning, 0 after, same clamp pattern as the other
   outputs). `x_cg` is unrelated to this wiring, it already feeds `AssembleRCG`
   separately (see item 3).

   `vre_flag` ended up back ON (an earlier attempt to leave it off broke the
   block's internal momentum computation, division by zero, see below).
   `Vre` is fed a `[0;0;0]` Constant (consistent with this project's decision
   not to model exhaust velocity separately, see Physics conventions below);
   `m_dot` is wired to the real analytic output, since the block genuinely
   requires it internally for its variable-mass momentum equation even when
   `Vre`'s own contribution is zero. `vre_flag`'s origin is still unknown,
   possibly one of Umut's settings from before the Phase A pivot.

   **Incident: `mass`/`I_dot`/`I`/`m_dot` arrived as 0 at t=0, dividing by
   zero inside the 6DOF block, for a subtle routing reason, not a logic bug.**
   Seven other hypotheses were tested and ruled out with hard evidence first
   (Stateflow `InitialValue`, chart is stateless, no effect; sample time,
   already continuous everywhere; algebraic loop, ruled out by forcing
   `AlgebraicLoopMsg='error'`, both `Update Diagram` and `sim()` passed clean;
   `ExecuteAtInitialization`, doesn't exist on this chart type;
   `UnderspecifiedInitializationDetection`, already `'Simplified'`; conditional
   subsystem execution, both `Forces and Moments` levels are virtual, cannot
   be conditional; duplicate/mis-scoped Goto tags, none, every tag in the
   model is unique). The real cause, found via `get_param(block,
   'CompiledSampleTime')`: the 4 new `From` blocks compiled to `[Inf Inf]`
   (Constant) while their `Goto` counterparts were correctly `[0 0]`
   (continuous), because those Gotos lived two virtual-subsystem levels deep,
   wired directly to `MassInertiaModel`'s raw output pins, and relied on
   Goto/From tag "local" visibility alone to jump both levels straight to
   root. See the routing convention rule in Style & engineering preferences
   below, this is now fixed by following that rule instead. Verified after
   the fix: all 4 `From` blocks compile to `[0 0]`, `Update Diagram` clean,
   3-second `sim()` completes with zero errors, `m(t)` decreases correctly
   (0.850 -> 0.8425 over 3s, matching `m_dot`), `Xe`/`Euler`/`DCM` show zero
   NaN/Inf, `DCM` stays properly bounded in [-1,1], and Phase A's `r_cg(0) =
   -0.10 m` still matches exactly (this routing fix touched nothing in the
   x_cg(t)/AssembleRCG path).
10. **cwd fragility on `matl.m`/InitFcn, documented, not fixed - now requires
   PROJECT root instead of `UMUT/`.** See `NOTES.md` for the full writeup
   (written when the constraint was still about `UMUT/`; the mechanism is
   unchanged, only the required folder moved). Short version:
   `rocket_upwork.slx`'s InitFcn is the bare string `matl;`, which only
   resolves if MATLAB's cwd is the PROJECT root at load/update time (same
   constraint on `matl.m`'s own `lqr_gain_design;` call and its CSV
   `readtable` calls, all of which use `fileparts(mfilename('fullpath'))`
   relative to `matl.m`'s own location, now the PROJECT root). The `UMUT/`
   subfolder was removed this session (client preference for a single
   project folder, not a fix to this underlying fragility) - `matl.m`,
   `lqr_gain_design.m`, `rocket_upwork.slx`, and the thrust CSVs all now sit
   directly in the PROJECT root alongside `CLAUDE.md`/`NOTES.md`/etc.
   Verified working from the new location: `open_system` + `Update Diagram`
   + a 40s `sim()` all succeeded with cwd = PROJECT root.
11. **`rocket.t_burn_ascent` was 2.5 s, unexplained, fixed to 10 s, then later
   superseded to be DERIVED from the real thrust curve (~21.5 s), DONE,
   verified, saved.** Pre-existing bug, unrelated to the mass/CG
   reconciliation task it was found during, fixed opportunistically because
   it lives in `Thrust Status`, the chart later touched for per-motor thrust
   work (item 12). Client-confirmed ascent fuel burn time was 10 s at the
   time; the file had 2.5 s with no on-disk history to explain why.
   Confirmed via chart-code tracing, not assumed, that nothing downstream
   depended on the wrong value: `Thrust Status`'s phase transitions are
   altitude/velocity-triggered (`h<=ignition_altitude` for the
   descent-ignition transition, see item 12's `ignition_altitude`
   parameterization), not offset from `ascent_burn_time` by a fixed
   duration.
   **Superseded this session, per explicit client instruction:**
   `rocket.t_burn_ascent` is no longer a manually chosen number at all - it
   is now `rocket.ascent_thrust_curve_t(end)`, i.e. whatever the last
   timestamp in `thrust_data_ascent_clean.csv` happens to be (currently
   21.5s, since the ascent curve is no longer truncated to a manually
   picked window, see item 12). This means the ascent burn duration now
   updates automatically whenever the client replaces the CSV with new
   static-test data - no code change needed. Verified fresh via
   `clear rocket; matl;`: `rocket.t_burn_ascent = 21.500`.
12. **Per-motor thrust, DONE, verified, saved.** New MATLAB Function block
   `PerMotorThrust` (in `Subsystem`, alongside `Thrust Status`) replaces the
   old `Gain1=[1;1;1]` scalar broadcast between `Thrust Status` and
   `rocket_forces_moments`. Output is `[3x1]` column, not `[1x3]` row - the
   downstream `Gain2` does elementwise `[1;1;1].*u`, a row vector causes a
   `PortDimsMismatch` compile error, confirmed by hitting this live. New
   parametric fields in `matl.m`: `rocket.thrust_pert = [0 0 0]` (fractional
   per-motor thrust multiplier, e.g. 0.05 = +5%), `rocket.ignition_delay =
   [0 0 0]` (seconds, per-motor ignition delay relative to shared phase-
   start). Both default to zero, symmetric baseline, reproduces pre-change
   behavior exactly. `time_in_phase` is computed inside the chart, not
   passed in: `remaining_fuel_time_s` (`Thrust Status`'s third output) only
   counts down meaningfully during phase 3 (descent burn); during phase 1
   (ascent) it is hardcoded to `descent_burn_time`, not usable for ascent
   timing. Ascent phase starts at simulation t=0 by construction (`Thrust
   Status` initializes to phase 1 on its first evaluation), so for phase==1,
   `time_in_phase = t` directly; for phase==3,
   `time_in_phase = descent_burn_time - remaining_fuel_time_s`. `phase==3`
   confirmed to be the descent phase via chart-code inspection, not assumed.
   Ascent motors: real static test data in use via
   `rocket.ascent_thrust_curve_t`/`_N`, loaded from
   `thrust_data_ascent_clean.csv` (renamed from `thrust_data_clean.csv`
   this session, see Repository layout), converted kgf->N (`*9.81`). This is
   RAW scale-reading data, NOT corrected for motor mass loss during the burn
   (the scale under-reads true thrust as the motor gets lighter over time,
   per the client's own framing), used as-is per the client's simplicity
   directive for M1. Descent motors: unchanged, still flat
   `rocket.T_nominal = 8.0 N` placeholder. All eML/struct-resolution inputs
   (curve arrays, `T_nominal`, `thrust_pert`, `ignition_delay`) passed in as
   explicit ports fed by Constant blocks, same pattern as item 6.

   **Updated this session, per explicit client instruction:** the ascent
   curve was originally sliced to the first 10 s (project history,
   2026-08-17: "use approximately the first 10 seconds of the measured
   thrust profile"). That truncation was REMOVED - `matl.m` now loads
   `thrust_data_ascent_clean.csv` in full (the real motor burn runs to
   ~21.5 s), and `rocket.t_burn_ascent` is derived directly from the
   curve's own last timestamp instead of a manually chosen number (see item
   11). `PerMotorThrust`'s `interp1(...,'linear','extrap')` clamp against
   `ascent_thrust_curve_t(end)` needed no code change, it already tracked
   the curve's real length automatically.

   Also new this session: `thrust_data_descent_clean.csv` (currently a
   duplicate of the ascent data, see Repository layout) is loaded into
   `rocket.descent_thrust_curve_t/N`, prepared as a drop-in slot for real
   descent motor data, but **deliberately NOT wired into `PerMotorThrust`
   yet** - doing so would silently replace the flat `T_nominal` placeholder
   with a declining curve shape/magnitude for the descent/landing phase,
   a real physics change that should be a separate, flagged decision, not a
   side effect of file prep. Descent phase in `PerMotorThrust` still uses
   `T_nominal` exactly as before.

   Also new this session: `Thrust Status`'s descent-ignition altitude
   threshold (`ignition_altitude_m = 15`, previously hardcoded inside the
   chart script) is now an explicit `ignition_altitude` input port, fed by
   a new `IgnitionAltitude` Constant block (`Value =
   rocket.descent_ignition_altitude_m`) inside `Subsystem`, same
   single-hop-Constant pattern as `Constant`/`Constant1`/`Constant2`
   feeding `ascent_burn_time`/`descent_burn_time`/`nominal_thrust`. No
   behavior change (still 15 m), just no longer hardcoded outside `matl.m`.
13. **Launch-pad ground reaction physics, DONE, verified, saved.** Item 12's
   real thrust curve exposed a genuine physics gap that did not exist with
   the old flat-8N placeholder: per-motor thrust takes ~1-1.5s to exceed the
   weight-equivalent threshold after ignition, during which the model
   previously free-fell through the ground (down to -11m, uncorrected)
   before the touchdown detector falsely fired at t~0. Fixed at the root,
   not by gating the detector alone (see `LiftoffArm` below for the detector
   side). New `GroundReaction` block, inserted into the `F_total` summing
   path at the `Forces and Moments` level (parallel to how
   `F_grav`/`F_aero`/`F_thrust` are already summed inside
   `rocket_forces_moments`; does NOT modify `rocket_forces_moments` itself).
   While `h<=0.001m`, cancels any net-downward component of `F_total` on the
   x_b axis exactly (models the physical launch rail holding the vehicle);
   zero effect once airborne or once thrust exceeds weight. Axis convention
   verified numerically, not assumed: `x_b` is the vehicle's confirmed
   vertical/thrust axis (99.6% weight at the `eul_0` ~85 deg initial
   attitude, via `DCM_be` applied to a unit NED-down vector; matches
   `rocket_forces_moments`'s own thrust formula, which applies `T(i)` purely
   along +x_b at zero gimbal deflection); the small 8.7% z_b cross-coupling
   is left unconstrained by design (spec calls for holding only the vertical
   axis, not all lateral force). `h = -Xe(3)` (NED, `Xe(3)` positive-down),
   traced block-by-block via `Xe -> Demux(port 3) -> Gain(-1) -> h`, not
   assumed. Verified via isolated synthetic tests (3 cases: grounded +
   net-down force cancels exactly, grounded + net-up force untouched,
   airborne + any force untouched) before wiring into the live model, a
   discipline worth repeating for future risky additions (see Style &
   engineering preferences). Residual dip after `GroundReaction`: only
   -0.043m (down from -11m), caused by the small uncancelled z_b gravity
   component since initial pitch is 85 deg, not exactly 90 deg; expected and
   correct given the x_b-only constraint. New `LiftoffArm` latch gates the
   existing `Hit Crossing`/`Stop Simulation` touchdown detector so it only
   becomes active once `h` first exceeds 1.0 m (persistent-state latch,
   matches `Thrust Status`'s own style; never resets once armed, even if
   altitude dips again later, e.g. a hover/landing approach). Deliberately
   not a fixed numeric tolerance on `Hit Crossing` itself: the upcoming
   sensitivity sweep varies `ignition_delay`/`thrust_pert`, which directly
   affects ground-phase dip depth, so a fixed tolerance could silently
   misfire on a future sweep point with a deeper residual dip than today's
   -0.043m. 1.0m threshold justified with data, not just accepted: >20x the
   worst observed dip, and only ~0.6s after real liftoff in the nominal
   case (h crosses 0 at t~1.35s, crosses 1.0m at t~1.98s), against a
   ~14-18s flight. **Gotcha for future Stateflow-adjacent additions:**
   `LiftoffArm` initially failed to compile
   (`Stateflow:Runtime:IllegalPersistentVarInContinuousTimeChart`), same
   class of bug as item 9's incident, because it inherited a continuous
   sample time. Root cause different from item 9 though: `Thrust Status`
   happens to inherit a discrete rate transitively via the FlightGear
   animation chain (confirmed `CompiledSampleTime` = 1/3000s), but
   `LiftoffArm`'s only input (`h`) traces straight to the continuous 6DOF
   integrator with nothing discrete in between. Fixed by explicitly setting
   `SystemSampleTime=0.001` on the block rather than relying on incidental
   inheritance. Do not assume a new Stateflow-adjacent block will
   automatically inherit a safe discrete rate just because a sibling block
   does; check `CompiledSampleTime` explicitly.
14. **Model is numerically chaotic near touchdown/hard impact, confirmed
   twice, methodology finding for the upcoming sensitivity sweep, not yet
   acted on.** Confirmed independently in both the item 1 (6DOF migration)
   and item 13 (`LiftoffArm`) verification work: re-running the SAME model
   with only a tiny numerical difference (solver `RelTol`, or an unrelated
   latch addition that shouldn't touch dynamics at all) produces
   meter-scale/tens-of-m/s-scale differences in apogee and touchdown values,
   while early-flight and pad-phase numbers stay stable to near machine
   precision (~1e-9 to 1e-11 level). **Implication for the M1 sensitivity
   study** (ignition delay + thrust variation sweep, one-factor-at-a-time,
   still not yet implemented): evaluate the "max angular deviation" metric
   during the ascent/active-control phase, NOT near touchdown, since
   late-flight chaos would swamp any real signal from the swept parameter
   with numerical noise unrelated to it.
15. **Full-flight baseline numbers below are STALE as of the parameter-
   consolidation session** (ascent burn duration changed from a manual 10s
   to the real ~21.5s curve, item 11/12) - liftoff timing, apogee, and
   touchdown will all differ from these figures and have not been
   re-measured yet. Re-derive before relying on these for anything
   quantitative. Original baseline (default params, items 1, 3/5/7/8/11
   (mass/CG), 12 (per-motor thrust), 13 (ground reaction) together, ascent
   burn = 10s): liftoff arms ~t=1.98s, apogee ~120-130m @ t~13.7-14.6s
   (varies run to run per item 14), touchdown ~66-77 m/s (pre-existing
   hard-landing behavior, unrelated to that session's work, expected to be
   addressed in a future milestone, not M1). A 40s `sim()` after this
   session's changes (mass/inertia/thrust-status parameterization, ascent
   curve un-truncated, UMUT folder consolidation) completed with no errors,
   but apogee/touchdown were not re-extracted. Full flight now reaches
   touchdown at t~24.3s (was ~14-20s range before today; consistent across
   the solver-config change in item 16, both hit t=24.321s almost exactly).
16. **Solver was badly mismatched to the problem, fixed, DONE, verified,
   saved.** User reported the Simulink "Play" button feeling very slow.
   Measured, not assumed: default config (`ode15s`, stiff/implicit,
   `RelTol=1e-6`) took 249s wall-clock for 24.3s of simulated flight,
   319,636 solver steps (average step ~76 microseconds - far finer than
   the configured 1ms `MaxStep`, meaning the solver was choosing tiny steps
   on its own via local error control, not hitting a cap). Root cause is
   NOT primarily solver stiffness choice - switching to `ode45` alone only
   helped partially (180s, 138,762 steps), proving the dominant cost was
   the `1e-6` tolerance being far tighter than this model needs (likely
   driven by the interpolated thrust curve's linear-segment derivative
   kinks every ~0.5s, plus threshold-based force cutoffs like
   `GroundReaction`/`LiftoffArm` that aren't flagged as zero-crossings, both
   of which make a tight-tolerance variable-step solver repeatedly shrink
   its step). `ode45` + `RelTol=1e-4` together: 30s wall-clock, 25,227
   steps - an ~8x speedup, reaching the identical final time (24.321s) as
   the original tight-tolerance run, so no accuracy loss observed for this
   flight. Applied directly to the model's saved configuration
   (`set_param('rocket_upwork','Solver','ode45')`,
   `RelTol='1e-4'`) since that's what the Play button actually uses, not
   left as a one-off `SimulationInput` override. `MaxStep=0.001` left
   unchanged (not the binding constraint, harmless as a safety bound). Not
   otherwise investigated: whether adding explicit zero-crossing detection
   to `GroundReaction`/`LiftoffArm` would help further - deferred, not
   necessary given the 8x win already achieved. User separately switched the
   model to a fixed-step `ode4`/`0.001s` solver afterward - also verified
   fast (15.85s wall-clock for 24.3s of flight), left as-is, no reason to
   revert.
17. **InitFcn broke again, second stray-workspace-variable incident, fixed at
   the root this time, DONE, verified, saved.** After item 16, the user hit
   a NEW failure: `Update Diagram`/InitFcn threw `Array indices must be
   positive integers or logical values` from `lqr_gain_design.m` line 17
   (`Q = diag([20, 20, 20, 0.5, 0.5, 0.5]);`). Root cause confirmed via
   `exist('diag','var')`/`which diag -all`: a stray variable literally named
   `diag` was sitting in the base MATLAB workspace, shadowing the builtin
   `diag()` function - since `matl.m`/`lqr_gain_design.m` run as scripts,
   they share whatever is in the base workspace, so `diag(...)` tried to
   *index* that variable instead of calling the function. Same root class
   of bug as the item (see Repository layout) `lqr_gain_design.m` fix
   earlier this session (stale `rocket.Ixx`), different stray variable, no
   known origin (not created by anything in this repo's own scripts).
   Fixed at the root this time instead of just clearing the one bad
   variable: added `clearvars` as the first executable line of `matl.m`, so
   InitFcn always starts from an empty workspace regardless of what junk is
   left over from prior runs/experiments. Verified the fix actually works,
   not just that the immediate symptom went away: deliberately poisoned the
   workspace again (`diag = 'poison'; rocket.Ixx = 999;`) before calling
   `matl;` and confirmed it still built `rocket` and updated the diagram
   cleanly. Safe because `matl.m` never depends on any pre-existing
   workspace variable - it rebuilds `rocket` entirely from scratch every
   time, and this project's own convention is to avoid flat workspace
   variables in favor of the `rocket` struct (see Legacy section).

## Physics / math conventions
- Full variable-inertia Euler equation: `M = I*omega_dot + I_dot*omega + omega x (I*omega)`.
  The `I_dot*omega` "phantom torque" term must be an explicit separate output from
  MassInertiaModel, not folded in.
- `Custom Variable Mass 6DOF` needs I(t) and dI/dt(t) as independently, physically
  consistent inputs — both derived from the same linear depletion model.
- Quaternion error vector part for attitude error, not Euler angle subtraction.
- No separate exhaust-velocity port needed; thrust curve already encodes propulsive
  force, no double-counting risk.
- Body axis convention, confirmed numerically not assumed (see Known gaps
  item 13): `x_b` is the vehicle's longitudinal/thrust axis, positive x_b is
  "up" (opposes gravity) at the nominal near-vertical launch attitude, and
  `rocket_forces_moments` applies per-motor thrust `T(i)` purely along +x_b
  at zero gimbal deflection. `h` (altitude) `= -Xe(3)` in the flat-Earth/NED
  frame (`Xe(3)` positive-down).

## Style & engineering preferences
- **Every editable parameter belongs in `matl.m`, never hardcoded inside a
  MATLAB Function/Stateflow chart script.** Explicit client directive
  (2026-08-26): "as long as we keep every parameter in a single main file
  matl.m (not hid into a hardcoded variable inside a function) - this
  applies for every variable/parameter." Applies going forward to any new
  plant-side block; feed values in as explicit input ports fed by Constant
  blocks with `Value = rocket.<field>` (see item 6 in Known gaps for why -
  eML/Stateflow chart scripts cannot resolve `rocket.*` struct fields
  internally). Does not apply to the controller (Umut's TVC/servo
  subsystems), which is explicitly off-limits for now, see Ownership split.
- Prefer native Simulink visual blocks over MATLAB Function blocks. Reserve MATLAB
  Function blocks for logic that genuinely needs it (e.g. cross-product loops).
- No persistent variables in MATLAB Function blocks under continuous solvers
  (causes errors) — keep stateless where possible.
- Modular architecture: gravity, thrust, drag, mass dynamics as separate blocks.
- Clean simple baseline before complexity (motor variation, sensitivity studies
  come after Milestone 1 core plant viability).
- All code comments in English.
- Never use em-dashes in any generated text or comments.
- When investigating a Simulink error: propose a step-by-step isolation method
  (e.g. isolate rigid-body dynamics from actuator) rather than guessing blindly.
- **Goto/From routing across subsystem levels: never jump more than one level
  in a single tag, even across virtual (non-atomic) subsystems.** A Goto tied
  directly to a raw block pin two or more virtual-subsystem levels below its
  matching From can compile to a `Constant` sample time (`CompiledSampleTime
  = [Inf Inf]`) instead of correctly inheriting the source's real sample time
  (e.g. `[0 0]` for continuous) - Simulink does not error on this, it silently
  produces a signal that is computed once and held forever. Confirmed as the
  root cause of a division-by-zero incident during Phase B (see known gap
  item 9). Every existing working Goto/From pair in this model (`F_total`,
  `M_total`, `DCMbe`, `Xe`, `Euler`, `Ve`, `Omega_be`) follows the same
  pattern instead: a real `Outport` at every intermediate subsystem level
  (plain wire, one hop each), and the Goto/From tag mechanism used only for
  the single, final hop immediately adjacent to the signal's true source
  (e.g. `Goto12` sits at root, fed directly by `blk_248`'s own `F_total`
  Outport, not nested inside it). Follow this exact pattern for any new
  cross-hierarchy signal: add a real `Outport` at each subsystem boundary the
  signal must cross, and reserve Goto/From tags for the last, adjacent hop
  only. After adding any new Goto/From pair, verify with
  `get_param(fromBlock, 'CompiledSampleTime')` that it matches the source's
  sample time - do not assume a clean `Update Diagram` means the routing is
  correct, this failure mode does not raise a compile error or diagnostic.
- **When replacing a block whose port count/order changes, wire and verify
  every connection by signal name, never by raw port number.** Confirmed
  necessary the hard way during the item 1 6DOF migration: the new block's
  `q` output is inserted at position 4, shifting `DCMbe`/`Vb`/`omega_be`
  down one index versus the old block. A position-based rewire would have
  silently cross-wired `Vb` and `DCMbe`.
- **Before wiring a new force/logic-summing block into the live model, build
  and run it as an isolated synthetic test first** (a standalone script
  exercising just the new block's logic against a handful of representative
  input cases, before any Simulink wiring). Used successfully for
  `GroundReaction` (item 13): three synthetic cases (grounded+net-down,
  grounded+net-up, airborne) caught the logic was correct before spending
  time on live wiring, and gave a clean baseline to compare the live-wired
  behavior against afterward. Worth repeating for any future risky addition.
- **A new MATLAB Function / Stateflow-adjacent block is not guaranteed to
  inherit a safe discrete sample time just because a sibling block does.**
  `Thrust Status` happens to inherit a discrete rate transitively via the
  FlightGear animation chain; `LiftoffArm` (item 13), added later with only
  an `h` input tracing straight to the continuous 6DOF integrator, inherited
  continuous and failed to compile with a persistent-variable error (same
  failure class as the item 9 Phase B incident, different root cause). If a
  new block uses persistent state, check its `CompiledSampleTime` explicitly
  after compiling rather than assuming; set `SystemSampleTime` explicitly if
  it resolves to continuous.

## Tooling notes
- MATLAB MCP server (`shareMATLABSession`) gives live access to open models —
  prefer this over static .slx/JSON export parsing when the session is available.
- Goto/From tag routing cannot be resolved from JSON netlist exports; needs live
  session or opening the .slx directly.
- MATLAB Function blocks are internally Stateflow EMChart objects — source lives
  in `.Script` via `sfroot()`, not `get_param`.

## Relevant Agentic Toolkit skill groups (already enabled)
MATLAB: aerospace, control-systems, math-and-optimization, matlab-programming,
matlab-software-development.
Simulink: control-systems, model-based-system-engineering, simulink-modeling,
simulink-simulation, verification-validation-and-test.