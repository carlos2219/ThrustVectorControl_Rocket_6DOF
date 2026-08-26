% Body-from-Earth reference DCM, ~89.9 deg pitch launch attitude target.
% Design choice, not measured. Not wired live: the TVC controller uses a
% hardcoded Constant instead, and lqr_gain_design.m defines its own,
% exact-90-deg DCM_ref locally. See CLAUDE.md known gap item 2.
rocket.DCM_ref = [0.001745328365898, 0, -0.999998476913288;
                  0, 1, 0;
                  0.999998476913288, 0, 0.001745328365898];

% Static aero reference CG [x,y,z] from nose tip, m. Feeds the aero
% CP-CG moment arm calc only, separate from the dynamic x_cg(t) used for
% the propulsion moment arm.
rocket.cg = [1.09, 0, 0];
% Static aero center of pressure [x,y,z] from nose tip, m. Placeholder,
% not measured.
rocket.cp = [0.88, 0, 0];

% Axial CG travel over burn, from nose tip, positive aft. x_cg_initial is
% measured (balancing, fully fueled). x_cg_burnout is derived assuming
% propellant burns from the motor pivot location (simplifying assumption
% for M1, not the true propellant centroid):
%   x_cg_burnout = (m_total*x_cg_full - m_prop_total*x_motor_pivot) / m_dry
%                = (2000*1090 - 690*1190) / 1310 = 1037.3 mm
rocket.x_cg_initial = 1.090; % m, measured
rocket.x_cg_burnout = 1.0373; % m, derived

% Aero coefficients (6x1, dimensionless), all zero - aero forces/moments
% are placeholder/disabled for now.
rocket.coeff = zeros(6,1);

% Vehicle body diameter, m. Parameters not final, rocket under construction.
rocket.diameter = 0.065;

% m_dry = m_total - m_prop_total = 2000 - 690 = 1310 g (measured). Already
% includes the 3 empty motor casings. Must match MassInertiaModel's
% hardcoded m_dry (CLAUDE.md known gap on duplicate parameter sets).
rocket.m_dry = 1.310;
% Casing mass alone, already implicit in m_dry above (not added again in
% m0_computed). Kept for future parallel-axis inertia work.
rocket.m_casing_each = 0.170;
rocket.n_engines = 3;

% Per-motor propellant mass, kg, client-measured. Informational only -
% PerMotorThrust uses the real thrust curve directly, and
% MassInertiaModel uses its own local m_prop total.
rocket.m_prop_ascent_each = 0.115;
rocket.m_prop_descent_each = 0.115;
% Per-motor burn rate, kg/s. Informational only, not consumed live.
rocket.burn_rate_each = 0.230 / 20;

% Client-confirmed ascent fuel burn time (was a stale 2.5s, unexplained).
rocket.t_burn_ascent = 10;
% Descent motor burn duration, s, client-confirmed.
rocket.t_burn_descent = 10;

% Total initial (fueled) mass. m_dry already includes casings, not added
% again here.
rocket.m0_computed = ...
    rocket.m_dry + ...
    rocket.n_engines * ...
    (rocket.m_prop_ascent_each + rocket.m_prop_descent_each);

rocket.m = rocket.m0_computed;

% LQR-facing inertia diagonal, kg*m^2, used by lqr_gain_design.m. Separate
% from MassInertiaModel's own local inertia literals (CLAUDE.md item 8).
% Iyy/Izz measured (bifilar pendulum). Ixx still a placeholder guess.
rocket.Iyy_burn = 0.338;
rocket.Izz_burn = 0.338;
rocket.Ixx_burn = 0.018;

rocket.I_burn = [rocket.Ixx_burn 0 0; 0 rocket.Iyy_burn 0;0 0 rocket.Izz_burn];

rocket.Ixx_dry  = 0.002;  % kg*m^2, roll-axis inertia (about x_b, nose axis), empty
rocket.Iyy_dry  = 0.05;   % kg*m^2, pitch-axis inertia (about y_b), empty
rocket.Izz_dry  = 0.05;   % kg*m^2, yaw-axis inertia (about z_b), empty

rocket.I_dry = [rocket.Ixx_dry 0 0; 0 rocket.Iyy_dry 0;0 0 rocket.Izz_dry];

% Extra inertia contributed by full propellant load (rough approximation).
rocket.Ixx_prop = 0.0002;
rocket.Iyy_prop = 0.01;
rocket.Izz_prop = 0.01;

rocket.I_prop = [rocket.Ixx_prop 0 0; 0 rocket.Iyy_prop 0;0 0 rocket.Izz_prop];


% Sea-level reference altitude for the atmosphere model.
rocket.h_ref = 0;

% Axial position of the motor gimbal pivot from nose tip, m, measured.
rocket.engine_pivot_from_nose = 1.190;
% Static (t=0) engine pivot position relative to the static rocket.cg.
% Superseded by AssembleRCG's dynamic x_cg(t) for force mixing; only used
% below to build the static rocket.r_cg.
rocket.engine_pivot_x_from_cg = ...
    -(rocket.engine_pivot_from_nose - rocket.cg(1));

% Equilateral-triangle motor mount side length, m.
rocket.side_length = 0.043;
% Radial distance from centerline to each motor (circumradius).
rocket.r_arm = rocket.side_length / sqrt(3);
% Per-motor angular placement around centerline, deg.
rocket.azimuth_deg = [0, 120, 240];

% Static (t=0-only) per-motor moment-arm matrix, frozen at initial CG.
% Feeds only the TVC controller's allocation matrix, which therefore does
% not track CG shift during burn (rocket_forces_moments does, via the
% separate dynamic AssembleRCG path).
rocket.r_cg = zeros(3, rocket.n_engines);

for i = 1:rocket.n_engines
    azimuth_angle = deg2rad(rocket.azimuth_deg(i));

    rocket.r_cg(:,i) = ...
        [rocket.engine_pivot_x_from_cg;
         rocket.r_arm * cos(azimuth_angle);
         rocket.r_arm * sin(azimuth_angle)];
end

% Flat single-motor thrust placeholder, N. Still used for descent motors
% (client instruction) via PerMotorThrust and descent_tilt_lqr, and as
% the TVC allocation matrix's magnitude scale for both phases (ascent's
% allocation still scales off this flat value, not the real curve below).
rocket.T_nominal = 8.0;
% Total nominal thrust (all 3 motors), N. Informational only.
rocket.T_total_nominal = ...
    rocket.n_engines * rocket.T_nominal;

% Ascent motor thrust curve: real static test data (client motor firing
% video), first 10s per client instruction. Loaded from
% thrust_data_clean.csv, a pre-cleaned copy of thrust_data.csv (dead time
% and tare-drift artifact trimmed, time re-zeroed to ignition). Still raw
% scale-reading data, not corrected for motor mass loss during burn - used
% as-is per client's simplicity directive for M1. Descent motors keep the
% flat rocket.T_nominal placeholder.
thrustDataPath = fullfile(fileparts(mfilename('fullpath')), 'thrust_data_clean.csv');
thrustDataTbl = readtable(thrustDataPath);
ascentMask = thrustDataTbl.time_seconds <= 10;
rocket.ascent_thrust_curve_t = thrustDataTbl.time_seconds(ascentMask)';
rocket.ascent_thrust_curve_N = thrustDataTbl.scale_reading_kg(ascentMask)' * 9.81;

% Per-motor sensitivity parameters for PerMotorThrust, both zero by default
% (symmetric baseline, no artificial imbalance).
rocket.thrust_pert = [0 0 0];
rocket.ignition_delay = [0 0 0];

% Per-motor gimbal deflection limit, deg, ascent phase. Not wired live -
% tvc_controller_dcm hardcodes the same value directly instead of reading
% this field (duplicate parameter, see CLAUDE.md known gaps).
rocket.gimbal_limit_ascent_deg = [-10, 10];
% Collective gimbal cant angle limit, deg, hover/descent phase. Not wired
% live - Descent Throttle hardcodes the max (60) directly instead of
% reading this field.
rocket.gimbal_limit_hover_deg = [-15, 60];

lqr_gain_design;

% Servo actuator dynamics (Umut's design/tuning, not measured). wn/zeta
% feed the Servo Actuator State-Space A matrix directly:
% A = [0 1; -wn^2 -2*zeta*wn].
rocket.servo.wn = 30;
rocket.servo.zeta = 0.7;
rocket.servo.max_rate_dps = 90;
rocket.servo.delay_s = 0.02;







