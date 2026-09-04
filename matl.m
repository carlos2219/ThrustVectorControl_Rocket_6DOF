% matl.m - builds the `rocket` struct used by rocket_upwork.slx's InitFcn.
% Single source of truth for every editable plant parameter. Requires
% MATLAB's cwd set to the project root (see README.md).

clearvars   % script shares the base workspace - avoid inheriting stale vars

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

rocket.mass_burn_duration = 20;        % s, propellant depletion time (separate from t_burn_ascent/t_burn_descent)
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

% Static (t=0) moment arm for the TVC allocation matrix only; the plant
% tracks the burning CG dynamically instead (AssembleRCG).
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

%% Thrust configuration
% CSV columns: time_seconds, thrust_m1_N, thrust_m2_N, thrust_m3_N.
% Interpolation happens in Simulink (root Thrust subsystem), not here.
thrustDataDir = fileparts(mfilename('fullpath'));

curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_ascent_clean.csv'));
rocket.ascent_thrust_curve_t = curveTbl.time_seconds';
rocket.ascent_thrust_curve_N = [curveTbl.thrust_m1_N'; curveTbl.thrust_m2_N'; curveTbl.thrust_m3_N'];
rocket.t_burn_ascent = rocket.ascent_thrust_curve_t(end);   % tracks the CSV's last timestamp

rocket.descent_ignition_altitude_m = 40;    % m
rocket.t_burn_descent = 10;                 % s, client-confirmed

curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_descent_clean.csv'));
rocket.descent_thrust_curve_t = curveTbl.time_seconds';
rocket.descent_thrust_curve_N = [curveTbl.thrust_m1_N'; curveTbl.thrust_m2_N'; curveTbl.thrust_m3_N'];

%% Gimbal limits
rocket.gimbal_limit_ascent_deg = [-10, 10];
rocket.gimbal_limit_hover_deg = [-15, 60];

%% Controller design (LQR gain, computed offline)
lqr_gain_design;

%% Servo actuator dynamics (Umut's tuning, not measured)
rocket.servo.wn = 30;
rocket.servo.zeta = 0.7;
rocket.servo.max_rate_dps = 90;
rocket.servo.delay_s = 0.02;
