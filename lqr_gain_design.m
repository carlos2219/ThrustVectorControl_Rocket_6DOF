%% lqr_gain_design.m

DCM_ref = [0 0 -1;
           0 1  0;
           1 0  0];

rocket.Ixx_lqr = rocket.Ixx_burn;

A = zeros(6,6);
A(1:3,4:6) = -eye(3);

B = zeros(6,3);
B(4,1) = 1/rocket.Ixx_lqr;
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
rocket.lqr.DCM_ref = DCM_ref;