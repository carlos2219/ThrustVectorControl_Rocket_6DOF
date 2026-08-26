clc; 
clear;

%Rocket center of gravity (CG) and center of pressure (CP)
rocket.cg = [1.09, 0, 0];
rocket.cp = [0.88, 0, 0];

%Rocket axial CG travel over burn (TBD PLACEHOLDERS pending Kruthick's measured
%values). Measured from nose tip, positive aft. NOTE: not yet consumed by
%MassInertiaModel(t) in SIM_model, which currently duplicates its own local
%x_cg_initial/x_cg_burnout literals instead of reading this struct - see
%CLAUDE.md known gap on duplicate/inconsistent parameter sets, needs reconciling.
rocket.x_cg_initial = 0.50; % m, TBD - CG at ignition (full propellant)
rocket.x_cg_burnout = 0.46; % m, TBD - CG at burnout (empty)

%Rocket mass
rocket.m_dry = 0.600;
rocket.m_casing_each = 0.170;

%Rocket engine params
rocket.n_engines = 3;


%Rocket Inertia Matrix
rocket.Iyy = 0.338;
rocket.Izz = 0.338;
rocket.Ixx = 0.018;

rocket.d           = 0.065; %diameter meters
rocket.A_ref       = pi * (rocket.d / 2)^2; %m^2
rocket.L_rocket    = 0.80; %length meters
rocket.AR          = rocket.L_rocket / rocket.d;


% Drag Force
rocket.Cd = 0.4;
% Rho is computed via the ISA block

%Rocket Servomotor parameters
rocket.servo.wn = 30;
rocket.servo.zeta = 0.7;
rocket.servo.max_rate_dps = 90;
rocket.servo.delay_s = 0.02;