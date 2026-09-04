%% lqr_gain_design.m

A = zeros(6,6);
A(1:3,4:6) = -eye(3);

B = zeros(6,3);
B(4,1) = 1/rocket.Ixx_burn;
B(5,2) = 1/rocket.Iyy_burn;
B(6,3) = 1/rocket.Izz_burn;

% Roll is deliberately de-weighted relative to pitch/yaw: roll rotates the
% vehicle about its own thrust axis and doesn't tilt where thrust points,
% but its allocation moment arm (r_arm) is ~4x shorter than pitch/yaw's
% (engine_pivot_x_from_cg), so fighting roll errors hard burns gimbal
% budget out of proportion to any benefit and starves pitch/yaw of
% authority via the shared saturation clamp in tvc_controller_dcm. Swept
% empirically (see DEVELOPMENT_NOTES.md): boosting roll weight instead of
% cutting it made lateral drift ~2x worse, not better.
% Rate weights (states 4-6) also raised well above the angle-only sweep's
% first pass: low rate weight left residual pitch/yaw rate at ascent
% burnout (~32 deg/s), which then free-tumbles the vehicle during the
% unpowered coast phase (no thrust = no gimbal authority to damp it) and
% wrecks the descent burn's attitude before it even ignites. Rate=15
% brought burnout pitch rate down to ~12 deg/s without giving back the
% lateral-drift gain from the angle weights.
Q = diag([8, 120, 120, 0.25, 15, 15]);
R = diag([200, 30, 30]);

K = lqr(A, B, Q, R);

disp(K);
disp(eig(A - B*K));

rocket.lqr.A = A;
rocket.lqr.B = B;
rocket.lqr.Q = Q;
rocket.lqr.R = R;
rocket.lqr.K = K;