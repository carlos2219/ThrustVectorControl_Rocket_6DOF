% matl.m - builds the `rocket` struct used by rocket_upwork.slx's InitFcn.
% Single source of truth for every editable plant parameter (mass, geometry,
% CG/CP, propellant, thrust curves, servo tuning, LQR gains). Run with
% MATLAB's cwd set to the project root (see NOTES.md).

% Runs as a script (shares the base workspace), so a stray leftover
% variable with the same name as a builtin (e.g. `diag`) or an old `rocket`
% field can silently break this. Clear the slate every time so InitFcn
% never depends on whatever happens to already be in the workspace.
clearvars

% Body-from-Earth DCM, ~89.9 deg pitch launch attitude. Not measured, and not
% wired to the controller (see ENGINEERING_LOG.md known gap item 2).
rocket.DCM_ref = [0.001745328365898, 0, -0.999998476913288;
                  0, 1, 0;
                  0.999998476913288, 0, 0.001745328365898];

% Static aero CG/CP from nose tip, m. Separate from the dynamic x_cg(t) below.
rocket.cg = [1.09, 0, 0];
rocket.cp = [0.88, 0, 0]; % placeholder, not measured

% Axial CG travel over burn, m from nose tip. x_cg_initial measured;
% x_cg_burnout derived assuming propellant burns from the motor pivot.
rocket.x_cg_initial = 1.090;
rocket.x_cg_burnout = 1.0373;

rocket.coeff = zeros(6,1); % aero coefficients disabled for now
rocket.diameter = 0.065;   % m, not final

% Dry mass (measured), includes 3 empty motor casings.
rocket.m_dry = 1.310;
rocket.m_casing_each = 0.170;
rocket.n_engines = 3;

% Per-motor propellant, kg (client-measured).
rocket.m_prop_ascent_each = 0.115;
rocket.m_prop_descent_each = 0.115;
rocket.burn_rate_each = 0.230 / 20;

% Total propellant, kg - feeds MassInertiaModel.
rocket.m_prop_total = rocket.n_engines * ...
    (rocket.m_prop_ascent_each + rocket.m_prop_descent_each);

% Mass-model burn duration, s - feeds MassInertiaModel. Kept independent from
% t_burn_ascent/t_burn_descent below (one continuous depletion, not two burns).
rocket.mass_burn_duration = 20;

rocket.descent_ignition_altitude_m = 15; % m, feeds Thrust Status
rocket.t_burn_descent = 10;              % s, client-confirmed
% t_burn_ascent is set below, derived from the real thrust curve.

rocket.m0_computed = rocket.m_dry + rocket.n_engines * ...
    (rocket.m_prop_ascent_each + rocket.m_prop_descent_each);
rocket.m = rocket.m0_computed;

% LQR-facing inertia. Iyy/Izz measured (bifilar pendulum); Ixx still a guess.
rocket.Ixx_burn = 0.018;
rocket.Iyy_burn = 0.338;
rocket.Izz_burn = 0.338;
rocket.I_burn = [rocket.Ixx_burn 0 0; 0 rocket.Iyy_burn 0; 0 0 rocket.Izz_burn];

% Dry/prop inertia - feeds MassInertiaModel (I = I_dry + I_prop*(1-burned)).
rocket.Ixx_dry  = 0.002;
rocket.Iyy_dry  = 0.05;
rocket.Izz_dry  = 0.05;
rocket.I_dry = [rocket.Ixx_dry 0 0; 0 rocket.Iyy_dry 0; 0 0 rocket.Izz_dry];

rocket.Ixx_prop = 0.0002;
rocket.Iyy_prop = 0.01;
rocket.Izz_prop = 0.01;
rocket.I_prop = [rocket.Ixx_prop 0 0; 0 rocket.Iyy_prop 0; 0 0 rocket.Izz_prop];

rocket.h_ref = 0; % sea-level reference altitude for the atmosphere model

% Motor geometry, measured.
rocket.engine_pivot_from_nose = 1.190;
rocket.engine_pivot_x_from_cg = -(rocket.engine_pivot_from_nose - rocket.cg(1));

rocket.side_length = 0.043; % equilateral motor mount, m
rocket.r_arm = rocket.side_length / sqrt(3);
rocket.azimuth_deg = [0, 120, 240];

% Static (t=0) per-motor moment arm - feeds only the TVC allocation matrix.
% rocket_forces_moments tracks the CG shift dynamically via AssembleRCG instead.
rocket.r_cg = zeros(3, rocket.n_engines);
for i = 1:rocket.n_engines
    azimuth_angle = deg2rad(rocket.azimuth_deg(i));
    rocket.r_cg(:,i) = [rocket.engine_pivot_x_from_cg;
                        rocket.r_arm * cos(azimuth_angle);
                        rocket.r_arm * sin(azimuth_angle)];
end

% Flat thrust placeholder, N - used for descent motors and as the TVC
% allocation magnitude scale.
rocket.T_nominal = 8.0;
rocket.T_total_nominal = rocket.n_engines * rocket.T_nominal;

% Ascent thrust curve: real static-test data, used in full (no truncation).
% t_burn_ascent is derived from its last timestamp, so it tracks whatever
% data is in the CSV automatically.
thrustDataPath = fullfile(fileparts(mfilename('fullpath')), 'thrust_data_ascent_clean.csv');
thrustDataTbl = readtable(thrustDataPath);
rocket.ascent_thrust_curve_t = thrustDataTbl.time_seconds';
rocket.ascent_thrust_curve_N = thrustDataTbl.scale_reading_kg' * 9.81;
rocket.t_burn_ascent = rocket.ascent_thrust_curve_t(end);

% Descent thrust curve: loaded but NOT wired into PerMotorThrust yet (descent
% still uses the flat T_nominal above - current focus is ascent only).
% Currently a duplicate of the ascent data, a placeholder slot for real
% descent motor test data.
descentThrustDataPath = fullfile(fileparts(mfilename('fullpath')), 'thrust_data_descent_clean.csv');
descentThrustDataTbl = readtable(descentThrustDataPath);
rocket.descent_thrust_curve_t = descentThrustDataTbl.time_seconds';
rocket.descent_thrust_curve_N = descentThrustDataTbl.scale_reading_kg' * 9.81;

% Per-motor sensitivity params for PerMotorThrust (for later sweeps).
rocket.thrust_pert = [0 0 0];
rocket.ignition_delay = [0 0 0];

% Gimbal limits - not wired live, controller hardcodes these directly.
rocket.gimbal_limit_ascent_deg = [-10, 10];
rocket.gimbal_limit_hover_deg = [-15, 60];

lqr_gain_design;

% Servo actuator dynamics (Umut's tuning, not measured).
rocket.servo.wn = 30;
rocket.servo.zeta = 0.7;
rocket.servo.max_rate_dps = 90;
rocket.servo.delay_s = 0.02;
