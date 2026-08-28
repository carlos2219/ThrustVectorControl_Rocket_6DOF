% matl.m - builds the `rocket` struct used by rocket_upwork.slx's InitFcn.
% Single source of truth for every editable plant parameter. Run with
% MATLAB's cwd set to the project root (see README.md, Getting started).

% Runs as a script (shares the base workspace) - clear the slate every
% time so InitFcn never depends on leftover variables from a prior run.
clearvars

%% Aerodynamics (disabled) and geometry reference
rocket.cg = [1.09, 0, 0];    % static aero CG from nose tip, m
rocket.cp = [0.88, 0, 0];    % static aero CP from nose tip, m (placeholder, not measured)
rocket.coeff = zeros(6,1);   % aero coefficients, disabled for now (negligible at current speeds)
rocket.h_ref = 0;            % sea-level reference altitude for the atmosphere model, m

%% Mass
rocket.m_dry = 1.310;                  % dry mass, kg (measured, includes 3 empty motor casings)
rocket.n_engines = 3;

rocket.m_prop_ascent_each = 0.115;     % propellant per motor, kg (client-measured)
rocket.m_prop_descent_each = 0.115;
rocket.m_prop_total = rocket.n_engines * (rocket.m_prop_ascent_each + rocket.m_prop_descent_each);

rocket.mass_burn_duration = 20;        % s, one continuous depletion covering ascent + descent
                                        % (independent from t_burn_ascent/t_burn_descent below)
rocket.m0_computed = rocket.m_dry + rocket.n_engines * ...
    (rocket.m_prop_ascent_each + rocket.m_prop_descent_each);

% Axial CG travel over burn, m from nose tip.
rocket.x_cg_initial = 1.090;           % measured, full propellant
rocket.x_cg_burnout = 1.0373;          % derived (assumes propellant burns from the motor pivot)

%% Inertia
% Dry/prop breakdown feeds the variable-mass model: I(t) = I_dry + I_prop*(1-burned).
rocket.Ixx_dry  = 0.002;
rocket.Iyy_dry  = 0.05;
rocket.Izz_dry  = 0.05;
rocket.I_dry = diag([rocket.Ixx_dry, rocket.Iyy_dry, rocket.Izz_dry]);

rocket.Ixx_prop = 0.0002;
rocket.Iyy_prop = 0.01;
rocket.Izz_prop = 0.01;
rocket.I_prop = diag([rocket.Ixx_prop, rocket.Iyy_prop, rocket.Izz_prop]);

% Burn/LQR-facing inertia. Iyy/Izz measured (bifilar pendulum); Ixx still a placeholder.
rocket.Ixx_burn = 0.018;
rocket.Iyy_burn = 0.338;
rocket.Izz_burn = 0.338;

%% Motor mount geometry and per-motor moment arm
rocket.engine_pivot_from_nose = 1.190;      % measured, m
rocket.engine_pivot_x_from_cg = -(rocket.engine_pivot_from_nose - rocket.cg(1));

rocket.side_length = 0.043;                 % equilateral motor mount side, m
rocket.r_arm = rocket.side_length / sqrt(3);
rocket.azimuth_deg = [0, 120, 240];

% Static (t=0) per-motor moment arm - feeds only the TVC allocation matrix.
% The plant tracks the burning CG dynamically instead (AssembleRCG).
rocket.r_cg = zeros(3, rocket.n_engines);
for i = 1:rocket.n_engines
    azimuth_angle = deg2rad(rocket.azimuth_deg(i));
    rocket.r_cg(:,i) = [rocket.engine_pivot_x_from_cg;
                        rocket.r_arm * cos(azimuth_angle);
                        rocket.r_arm * sin(azimuth_angle)];
end

%% Attitude reference
% Body-from-Earth DCM, ~89.9 deg pitch launch attitude. Not measured.
rocket.DCM_ref = [0.001745328365898, 0, -0.999998476913288;
                  0, 1, 0;
                  0.999998476913288, 0, 0.001745328365898];

%% Thrust and burn timing
rocket.T_nominal = 8.0;   % flat thrust placeholder, N (descent motors + TVC allocation scale)

% Ascent thrust curve: real static-test data, used in full. t_burn_ascent is
% derived from its own last timestamp, so it tracks whatever data is in the CSV.
thrustDataDir = fileparts(mfilename('fullpath'));
curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_ascent_clean.csv'));
rocket.ascent_thrust_curve_t = curveTbl.time_seconds';
rocket.ascent_thrust_curve_N = curveTbl.scale_reading_kg' * 9.81;
rocket.t_burn_ascent = rocket.ascent_thrust_curve_t(end);

rocket.descent_ignition_altitude_m = 15;    % m
rocket.t_burn_descent = 10;                 % s, client-confirmed

% Descent thrust curve: loaded but not wired into PerMotorThrust yet - current
% focus is ascent only. Placeholder slot (duplicate of ascent data) until real
% descent motor test data is available.
curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_descent_clean.csv'));
rocket.descent_thrust_curve_t = curveTbl.time_seconds';
rocket.descent_thrust_curve_N = curveTbl.scale_reading_kg' * 9.81;

% Per-motor sensitivity knobs for PerMotorThrust (for later sweeps), zero by default.
rocket.thrust_pert = [0 0 0];
rocket.ignition_delay = [0 0 0];

%% Gimbal limits
rocket.gimbal_limit_ascent_deg = [-10, 10];
rocket.gimbal_limit_hover_deg = [-15, 60];

%% Artificial test thrust (quick sensitivity checks - see Manual Switches
% in Thrust Subsystem to flip between this and the real motor data).
rocket.test_ascent_thrust_N = 8;    % flat per-motor thrust, N (real curve peaks at ~9.2 N)
rocket.test_ascent_curve_N = rocket.test_ascent_thrust_N * ones(size(rocket.ascent_thrust_curve_N));
rocket.test_descent_thrust_N = 8;   % flat per-motor thrust, N (same as rocket.T_nominal)

%% Controller design (LQR gain, computed offline)
lqr_gain_design;

%% Servo actuator dynamics (Umut's tuning, not measured)
rocket.servo.wn = 30;
rocket.servo.zeta = 0.7;
rocket.servo.max_rate_dps = 90;
rocket.servo.delay_s = 0.02;
