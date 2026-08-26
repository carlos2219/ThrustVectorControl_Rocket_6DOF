# TVC Rocket 6DOF Model — Walkthrough

This is a guided tour of `rocket_upwork.slx`, written for someone
learning the model from scratch, in **physical/logical flow order**, not
implementation chronology. It is meant to be readable on its own, without
opening Simulink, as a first pass.

This is **not** a replacement for `ENGINEERING_LOG.md`. `ENGINEERING_LOG.md` is the engineering
log/diary: decisions, incidents, dates, verification numbers, "Known gaps"
with item numbers. This document explains *what the model does and why*, in
one linear read. Where something is a placeholder, simplification, or open
item, this document points at the relevant `ENGINEERING_LOG.md` "Known gaps" item
number rather than re-explaining it.

## 1. High-level signal flow

```
        ┌─────────────────────────────────────────────────────────────┐
        │                                                               │
        v                                                               │
  ┌───────────┐    F_total, M_total    ┌──────────────┐   Xe,Ve,DCMbe,  │
  │  PLANT    │────────────────────────>│  6DOF         │   Euler,      │
  │ (forces/  │   (body-frame N, N-m)   │  INTEGRATION  │───omega_be────┤
  │  moments) │                         │ (Quaternion)  │                │
  └───────────┘                         └──────────────┘                │
        ^                                       │                       │
        │ T (per-motor thrust)                  │ DCMbe, omega_be       │
        │ alpha, beta (gimbal angles)            │ (attitude + rate)    │
        │                                        v                      │
  ┌───────────┐    alpha_cmd, beta_cmd   ┌──────────────┐               │
  │ ACTUATOR  │<─────────────────────────│  CONTROLLER   │<──────────────┘
  │ (servo    │                         │ (LQR/DCM TVC) │
  │ dynamics) │                         └──────────────┘
  └───────────┘
```

One paragraph per stage:

- **Plant (forces/moments):** combines per-motor thrust (gimbal-rotated),
  aerodynamic force/moment, gravity, and (near the ground) a launch-pad
  reaction force into one net body-frame force `F_total` and moment
  `M_total` each timestep. Also computes the vehicle's time-varying mass and
  inertia as propellant burns.
- **6DOF integration:** takes `F_total`, `M_total`, and the current
  mass/inertia, integrates the rigid-body equations of motion (quaternion
  attitude representation), and outputs the vehicle's position, velocity,
  attitude, and angular rate.
- **Controller:** reads the vehicle's current attitude (`DCMbe`) and angular
  rate (`omega_be`), compares against a reference attitude, and computes
  corrective per-motor gimbal angle commands (`alpha`, `beta`) via an LQR
  law and thrust allocation.
- **Actuator:** models the physical gimbal servo's response (bandwidth, rate
  limit, delay) between the controller's *commanded* angles and the
  *physically realized* angles that actually deflect the motors, which feed
  back into the plant's thrust vector calculation.

## 2. Mass & inertia (`MassInertiaModel`)

Path: `Forces and Moments/Forces and Moments/MassInertiaModel`

As the 3 KNSB motors burn propellant, the vehicle gets lighter, its inertia
drops, and its CG shifts toward the nose. All four quantities are derived
from **one** linear depletion fraction, `frac_burned`, so they stay
physically consistent with each other:

```
frac_burned(t) = t / t_burn          (0 at ignition, 1 at burnout, then holds at 1)

m(t)     = m_dry + m_prop * (1 - frac_burned)
m_dot(t) = -m_prop / t_burn            while burning, else 0

Ixx(t) = Ixx_dry + Ixx_prop * (1 - frac_burned)   (same form for Iyy, Izz)
dIxx/dt(t) = -Ixx_prop / t_burn        while burning, else 0   (analytic derivative)

x_cg(t) = x_cg_initial + (x_cg_burnout - x_cg_initial) * frac_burned
```

`m_dry = 1.310 kg`, `m_prop = 0.690 kg` (client-measured, all fed in from
`matl.m`, see Known gaps item 7). `x_cg_initial = 1.090 m`,
`x_cg_burnout = 1.0373 m` from nose tip (item 5). `mass_burn_duration = 20 s`
is a single continuous ramp covering ascent + descent together - this chart
does not know about the separate ascent/descent phase split that
`Thrust Status` tracks (item 7).

`I(t)` is diagonal only, with **no parallel-axis correction** for the CG
shift `x_cg(t)` - a deliberate M1 simplification (item 8). `dI/dt` is
required as an independent input alongside `I` (not derived internally by
the 6DOF block) because the full variable-inertia Euler equation needs the
`I_dot*omega` "phantom torque" term explicitly:

```
M = I*omega_dot + I_dot*omega + omega x (I*omega)
```

`x_cg(t)` feeds `AssembleRCG`, which turns it into the axial component of
the per-motor moment arm:

```
r_cg(1,i) = x_cg(t) - engine_pivot_from_nose      (same for all 3 motors)
r_cg(2,i) = r_arm * cos(azimuth_deg(i))            (static, doesn't change with burn)
r_cg(3,i) = r_arm * sin(azimuth_deg(i))
```

## 3. Thrust generation

### Thrust Status (`Subsystem/Thrust Status`)

A vehicle-level flight-phase state machine (one CG trajectory, one phase -
not per-motor):

```
phase 1 (ASCENT)  -- t < t_burn_ascent --> phase 2 (COAST)
phase 2 (COAST)    -- descending & h<=ignition_altitude --> phase 3 (DESCENT BURN)
phase 3 (DESCENT)  -- propellant exhausted (t_burn_descent after ignition) --> phase 4 (DONE)
```

Phase 1→2 is **time-triggered**, but `t_burn_ascent` is not a fixed number -
it's derived from the real ascent thrust curve's own last timestamp
(currently ~21.5s, item 11). Phase 2→3 is **altitude/velocity-triggered**,
against a parameterized `ignition_altitude` (15m, item 12), not
time-triggered - this matters because `remaining_fuel_time_s` (this chart's
third output) only counts down meaningfully during phase 3; during phase 1
it's a constant, not usable for ascent timing.

### PerMotorThrust (`Subsystem/PerMotorThrust`)

Computes real per-motor thrust, replacing the old flat placeholder:

```
time_in_phase = t                                    (phase 1, ascent)
time_in_phase = t_burn_descent - remaining_fuel_time_s (phase 3, descent)

t_local(i) = time_in_phase - ignition_delay(i)

if phase==1 and t_local>=0:
    T(i) = interp1(ascent_thrust_curve_t, ascent_thrust_curve_N, t_local) * (1 + thrust_pert(i))
elseif phase==3 and t_local>=0:
    T(i) = T_nominal * (1 + thrust_pert(i))
else:
    T(i) = 0
```

Ascent motors follow the **real static-test thrust curve**
(`thrust_data_ascent_clean.csv`, kgf→N, used in full - item 12). Descent
motors still use the flat `T_nominal=8N` placeholder - current focus is
ascent only; `thrust_data_descent_clean.csv` exists as a prepared slot for
real descent data but isn't wired in yet. `thrust_pert` and
`ignition_delay` are per-motor sensitivity knobs for a later sensitivity
study, zero by default.

Output is `T`, a `[3x1]` **column** vector, not a row (a row vector causes
a port-dimension-mismatch compile error against the downstream gain).

## 4. Force/moment mixing (`rocket_forces_moments`)

Path: `Forces and Moments/Forces and Moments/MATLAB Function`

The body-frame force/moment mixer, combining all 3 motors' thrust with aero
and gravity:

```
F_i = T(i) * [cos(alpha(i))*cos(beta(i)); sin(alpha(i))*cos(beta(i)); sin(beta(i))]
M_i = cross(r_cg(:,i), F_i)

F_total = sum(F_i) + F_aero + F_grav
M_total = sum(M_i) + M_aero
```

`alpha`/`beta` are the per-motor pitch/yaw gimbal deflections from the
controller (section 7). At zero deflection, `F_i` reduces to `[T(i);0;0]` -
thrust purely along **+x_b**, confirmed to be the vehicle's vertical/thrust
axis (see section 5 for why this matters, and `ENGINEERING_LOG.md` Physics/math
conventions for the numerical verification).

`r_cg` comes from `AssembleRCG` (section 2) - the moment arm here **does**
track the burning CG. Note (audit finding, `matl.m` comments on
`rocket.r_cg`): the TVC controller's own thrust-allocation matrix uses a
**different, static** `r_cg` (frozen at t=0), not this dynamic one - a real
asymmetry, not fixed in this pass.

## 5. Ground/launch-pad physics

### GroundReaction (`Forces and Moments/GroundReaction`)

Solves a real physics gap the switch to real thrust data exposed: the
measured thrust curve takes ~1-1.5s to exceed the vehicle's weight after
ignition. Without this block, the vehicle free-falls through the ground
during that window before the touchdown detector falsely fires
(`ENGINEERING_LOG.md` item 13).

```
if h <= 0.001:
    F_reaction(x_b) = -min(F_total_pre(x_b), 0)   # cancel net-downward x_b force exactly
    F_reaction(y_b, z_b) = 0                       # lateral force untouched
else:
    F_reaction = [0;0;0]                            # fully airborne, no effect
```

Only the vertical (`x_b`) axis is held - moments and lateral force pass
through untouched, so attitude correction still works normally while
grounded. This models a launch rail: it can only push up along the rail
axis, not sideways or rotationally.

### LiftoffArm (root level)

A separate concern from `GroundReaction` above: the model's touchdown
detector (`Hit Crossing`/`Stop Simulation`) has **zero tolerance**, so even
`GroundReaction`'s small residual pad-phase dip (~-0.043m, from the small
uncancelled lateral gravity component at the ~85° launch attitude) would
falsely trigger it. `LiftoffArm` is a one-way latch: touchdown detection
stays disabled until `h` first exceeds 1.0m, then stays permanently enabled
for the rest of the flight (including a later hover/landing approach).
Deliberately not a fixed tolerance on `Hit Crossing` itself, since the
ground-phase dip depth varies with `thrust_pert`/`ignition_delay` (item 13).

## 6. 6DOF integration

Block: `6DOF (Quaternion)` (root level), `aerolib6dof2/Custom Variable Mass
6DOF (Quaternion)`.

**Inputs:** `F_xyz` (=`F_total`, N), `M_xyz` (=`M_total`, N-m), `m` (kg),
`dI/dt` (kg·m²/s), `I` (kg·m²) - all body-frame, all from sections 2 and 4
above.

**Outputs:** `Ve` (Earth-frame velocity, m/s), `Xe` (Earth-frame position,
m), Euler angles `[phi,theta,psi]` (rad), `q` (quaternion, currently
unconnected/unused), `DCMbe` (body-from-Earth rotation matrix), `Vb`
(body-frame velocity, unused downstream), `omega_be` (body angular rate,
rad/s).

**Why quaternion, not Euler angles:** the vehicle's nominal flight condition
is near-vertical (~90° pitch). An Euler-angle representation has a
mathematical singularity (gimbal lock) exactly at 90° pitch; a quaternion
representation has no such singularity anywhere. The controller (section 7)
also computes its attitude error via DCM, not Euler subtraction, for the
same reason.

`mtype=Custom Variable Mass`: the block does **not** compute mass/inertia
internally - it trusts `m`, `dI/dt`, `I` fed in externally each step (from
`MassInertiaModel`), and requires them to be consistent with each other
(the `I_dot*omega` term in the Euler equation only cancels correctly if they
are). This block was migrated this session from a legacy masked block
(`aerolibobsolete/6DOF (Euler Angles)`) to this clean, current-Aerospace-
Blockset block - see `ENGINEERING_LOG.md` item 1 for the full port-diff and
verification writeup.

## 7. Controller / actuator (Umut's contribution)

High-level only - this is Umut's design; the goal here is to understand the
*interface*, not audit code someone else is responsible for.

**Attitude controller** (`TVC DCM Controller/MATLAB Function1`,
`tvc_controller_dcm`): reads the vehicle's current attitude (`DCMbe`) and
rate (`omega_be`), computes a DCM-based attitude error (a "vee-map" of the
skew-symmetric part of `DCM_be * DCM_reference'`, valid for small errors),
feeds `[attitude_error; rate]` into an LQR feedback law
(`M_cmd = -K * state_error`, `K` designed offline by `lqr_gain_design.m`),
then allocates that commanded moment across the 3 motors' gimbal axes via a
pseudo-inverse allocation matrix, clamped to gimbal limits. During descent,
a separate collective "throttle angle" (below) is added on top as a
feedforward term.

**Descent hover throttle** (`Descent Throttle/MATLAB Function2`,
`descent_tilt_lqr`): since the solid motors can't be throttled by reducing
chemical output, this function throttles *net vertical thrust* a different
way - commanding all 3 motors to cant outward together by a collective angle
`theta_throttle`, so vertical thrust becomes `T_total*cos(theta)` while each
motor still burns at full thrust. A simple altitude/velocity feedback law
adjusts this angle around a computed hover-equilibrium value. Only active
during phase 3 (descent burn).

**Servo actuator dynamics** (`Servo Actuator`, `Servo Actuator1`): native
Simulink State-Space + Transport Delay blocks per motor, modeling the real
gimbal actuator's bandwidth/damping/rate-limit/delay between the
controller's *commanded* `alpha`/`beta` and the *physically realized*
deflection that actually feeds `rocket_forces_moments`.

## 8. Known simplifications / open items

Pointers only - see `ENGINEERING_LOG.md` "Known gaps" for the full writeup on each:

- **Item 2:** three different `DCM_ref` values exist in this codebase, only
  one (a hardcoded literal in the TVC controller) is actually live.
- **Item 5:** `x_cg_burnout` is derived (propellant-concentrated-at-pivot
  assumption), not directly measured.
- **Item 7:** `MassInertiaModel`'s `mass_burn_duration=20s` is independent
  from `Thrust Status`'s phase-split timing, by convention only.
- **Item 8:** `I(t)` has no parallel-axis correction for CG shift. `Ixx`
  (LQR-facing) is still an unmeasured placeholder; `Iyy`/`Izz` are measured.
- **Item 11:** ascent burn duration is derived from the real thrust curve's
  own last timestamp (~21.5s), not a fixed number.
- **Item 12:** ascent thrust curve is raw scale-reading data, not corrected
  for motor mass loss during the burn; descent motors still use a flat
  placeholder, current focus is ascent only.
- **Item 13:** residual ~-0.043m pad-phase altitude dip is expected, not a
  bug (only the vertical axis is held by `GroundReaction`).
- **Item 14:** the model is numerically chaotic near touchdown/hard impact -
  don't expect exact apogee/touchdown reproducibility between runs that
  differ only in solver settings or unrelated additions.
- **Item 17:** `matl.m` runs `clearvars` first - InitFcn shares the base
  MATLAB workspace, and a stray leftover variable has broken it twice.
- **Other loose ends** (see `matl.m` comments): `rocket.gimbal_limit_ascent_deg`/
  `gimbal_limit_hover_deg` are unused, duplicated as hardcoded literals
  elsewhere; the TVC controller's thrust-allocation `r_cg` is static (t=0),
  not the dynamic burn-tracking one; several `rocket.*` fields
  (`engine_pivot_x_from_cg`, `T_total_nominal`, `m_prop_ascent_each`/
  `descent_each`, `burn_rate_each`) are informational/reference values not
  wired to any live block.
