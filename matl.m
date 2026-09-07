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

% NOTE: there used to be a separate rocket.Ixx_burn/Iyy_burn/Izz_burn here
% (0.018/0.338/0.338 - a bifilar-pendulum measurement), fed only into
% lqr_gain_design.m's B matrix and never reconciled with I_dry/I_prop
% above (0.06 vs 0.338 for pitch/yaw - ~5.6x off). That mismatch meant K
% was designed for a body ~5.6-8x "heavier" (rotationally) than the one
% actually simulated, which explains the growing pitch/yaw rate
% oscillation seen in the last ~1s of the ascent burn. Removed - the LQR
% design inertia is now derived directly from I_dry/I_prop in
% lqr_gain_design.m, so it can't drift out of sync with the plant again.

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

%% Environment
rocket.g = 9.81;    % m/s^2, single-sourced (used by descent ignition timing and hover throttle)

%% Attitude reference
% Not measured. 89.9 (not exactly 90) so the plant's initial condition
% (Rocket/6DOF (Quaternion) eul_0, set to deg2rad(rocket.launch_pitch_deg))
% and the controller's DCM_ref are always derived from this single value
% instead of drifting apart independently.
rocket.launch_pitch_deg = 89.9;
theta_launch = deg2rad(rocket.launch_pitch_deg);
rocket.DCM_ref = [cos(theta_launch), 0, -sin(theta_launch);
                  0,                 1,  0;
                  sin(theta_launch), 0,  cos(theta_launch)];

%% Thrust configuration
% CSV columns: time_seconds, thrust_m1_N, thrust_m2_N, thrust_m3_N.
% Interpolation happens in Simulink (root Thrust subsystem), not here.
thrustDataDir = fileparts(mfilename('fullpath'));

curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_ascent_clean.csv'));
rocket.ascent_thrust_curve_t = curveTbl.time_seconds';
rocket.ascent_thrust_curve_N = [curveTbl.thrust_m1_N'; curveTbl.thrust_m2_N'; curveTbl.thrust_m3_N'];
rocket.t_burn_ascent = rocket.ascent_thrust_curve_t(end);   % tracks the CSV's last timestamp

rocket.t_burn_descent = 10;                 % s, client-confirmed
rocket.hover_altitude_m = 1;                % m, below this the fine velocity-feedback throttle takes over

% Descent-motor ignition is no longer a fixed altitude - it's a real-time
% "suicide burn" trigger (client-proposed): ignite the instant the
% remaining altitude equals the distance needed to brake the current fall
% speed to a stop using full (untilted) descent thrust, plus a safety
% margin. This is computed in `Thrust Status` (needs live mass + gravity +
% nominal descent thrust, see below); the old fixed
% `descent_ignition_altitude_m` is gone (superseded, see DEVELOPMENT_NOTES.md).
% `descent_ignition_margin_m` is the extra stopping distance added on top
% of the bare-minimum brake distance, so real disturbances (wind, lateral
% correction stealing vertical thrust budget) don't eat the whole margin -
% the vehicle should still be moving slowly, not exactly at rest, when it
% reaches `hover_altitude_m`, leaving `descent_hover_K_V`/remaining burn
% time to finish the job. Swept 5-35 m against an 8-seed Monte Carlo (see
% DEVELOPMENT_NOTES.md): 20 m is the committed point (best mean vz, second
% -best worst-case vz, best worst-case lateral drift - non-monotonic
% neighbors beyond 25 m are the same touchdown-proximity chaos already
% documented elsewhere in this file, not a bug).
rocket.descent_ignition_margin_m = 20;

% Hover-throttle feedback gains (descent_tilt_lqr) - used to be hardcoded
% inside the MATLAB Function block, violating this project's own
% single-source-of-truth convention. K_V swept 0.20-0.55 (see
% DEVELOPMENT_NOTES.md): touchdown vz is a sharp, non-monotonic function
% of K_V near this value (a real minimum, not a smooth one - the
% touchdown-proximity chaos already documented elsewhere applies here
% too), so don't nudge this without re-sweeping and checking neighbors.
% K_H is now dead: the suicide-burn descent throttle law (see
% DEVELOPMENT_NOTES.md) replaced the old h_err-based formula that used it
% with a v_target-only fine-control law - kept only so descent_tilt_lqr's
% existing input port doesn't need removing.
rocket.descent_hover_K_H = -0.2;
rocket.descent_hover_K_V = -0.31;

% Lateral guidance. Nothing upstream of this closed a loop on horizontal
% (X/Y) position or velocity - the attitude controller only ever holds a
% fixed vertical DCM_ref, so any horizontal velocity picked up earlier in
% flight just carries the vehicle sideways for as long as it's still
% falling. This biases DCM_ref off-vertical during descent (phase 3
% only) toward killing X/Y position+velocity error, the same PD structure
% as descent_hover_K_H/K_V but for the two horizontal axes - see
% `lateral_guidance_dcm` in Controller. Swept against an 8-seed Monte
% Carlo (see DEVELOPMENT_NOTES.md): this is a real tradeoff against
% touchdown vz, not free improvement - these values were picked as the
% point where worst-case vz is still ~unchanged from no lateral guidance
% at all, while worst-case drift drops ~44%. Pushing further (higher
% gains/max_tilt) keeps improving drift but starts costing vz net
% negative - re-run that sweep before changing these.
rocket.lateral_guidance_K_pos = 0.015;
rocket.lateral_guidance_K_vel = 0.05;
rocket.lateral_guidance_max_tilt_deg = 20;

curveTbl = readtable(fullfile(thrustDataDir, 'thrust_data_descent_clean.csv'));
rocket.descent_thrust_curve_t = curveTbl.time_seconds';
rocket.descent_thrust_curve_N = [curveTbl.thrust_m1_N'; curveTbl.thrust_m2_N'; curveTbl.thrust_m3_N'];

% Total descent thrust at ignition (t'=0 on the descent curve), used by
% the suicide-burn ignition trigger to estimate available braking
% deceleration before the descent motors have actually fired.
rocket.descent_T_total_nominal = sum(rocket.descent_thrust_curve_N(:,1));

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
