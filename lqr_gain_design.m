%% lqr_gain_design.m

A = zeros(6,6);
A(1:3,4:6) = -eye(3);

B = zeros(6,3);
B(4,1) = 1/rocket.Ixx_burn;
B(5,2) = 1/rocket.Iyy_burn;
B(6,3) = 1/rocket.Izz_burn;

Q = diag([20, 20, 20, 0.5, 0.5, 0.5]);
R = diag([200, 200, 200]);

K = lqr(A, B, Q, R);

disp(K);
disp(eig(A - B*K));

rocket.lqr.A = A;
rocket.lqr.B = B;
rocket.lqr.Q = Q;
rocket.lqr.R = R;
rocket.lqr.K = K;