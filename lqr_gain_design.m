%% lqr_gain_design.m

% Design inertia: I(t) at ascent burnout, matching MassInertiaModel's own
% formula (I = I_dry + I_prop*(1-frac_burned)) exactly so this can't drift
% out of sync with the plant the way the old hardcoded rocket.Ixx_burn/
% Iyy_burn/Izz_burn (a bifilar-pendulum measurement, ~5.6-8x larger than
% the simulated body) did. Recompute here if I_dry/I_prop/t_burn_ascent/
% mass_burn_duration change - nothing below needs updating by hand.
frac_burned_at_ascent_burnout = min(rocket.t_burn_ascent / rocket.mass_burn_duration, 1);
I_burn = rocket.I_dry + rocket.I_prop * (1 - frac_burned_at_ascent_burnout);

A = zeros(6,6);
A(1:3,4:6) = -eye(3);

B = zeros(6,3);
B(4,1) = 1/I_burn(1,1);
B(5,2) = 1/I_burn(2,2);
B(6,3) = 1/I_burn(3,3);

% Roll is deliberately de-weighted relative to pitch/yaw: roll rotates the
% vehicle about its own thrust axis and doesn't tilt where thrust points,
% but its allocation moment arm (r_arm) is ~4x shorter than pitch/yaw's
% (engine_pivot_x_from_cg), so fighting roll errors hard burns gimbal
% budget out of proportion to any benefit and starves pitch/yaw of
% authority via the shared saturation clamp in tvc_controller_dcm. Swept
% empirically (see DEVELOPMENT_NOTES.md): boosting roll weight instead of
% cutting it made lateral drift ~2x worse, not better.
% Rate weights (states 4-6) also raised above the angle-only sweep's first
% pass: low rate weight left residual pitch/yaw rate at ascent burnout,
% which then free-tumbles the vehicle during the unpowered coast phase (no
% thrust = no gimbal authority to damp it) and wrecks the descent burn's
% attitude before it even ignites.
% Rate weight swept after the I_burn fix above: 20 is the best point in
% the well-behaved 10-20 band (all-real closed-loop eigenvalues, burnout
% pitch/yaw rate ~3 deg/s, no oscillation). NOT pushed higher: 25-40
% contains one clean-looking point (30) surrounded
% by roll-axis blowups (27, 29, 35 all show burnout roll rate 150-250+
% deg/s vs 10-20's 47-68 deg/s), driven by the shared gimbal-saturation
% clamp in tvc_controller_dcm - the same chaotic-near-threshold behavior
% documented elsewhere in this file/DEVELOPMENT_NOTES.md, just relocated
% to a different Q range now that the oscillation above is gone. See
% DEVELOPMENT_NOTES.md for the full sweep.
Q = diag([8, 120, 120, 0.25, 20, 20]);
R = diag([200, 30, 30]);

K = lqr(A, B, Q, R);

disp(K);
disp(eig(A - B*K));

rocket.lqr.A = A;
rocket.lqr.B = B;
rocket.lqr.Q = Q;
rocket.lqr.R = R;
rocket.lqr.K = K;
