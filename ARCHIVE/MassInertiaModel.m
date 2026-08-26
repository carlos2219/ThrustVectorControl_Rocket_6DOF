function [m, dIdt, I, r_cg_t] = MassInertiaModel(t)
% Computes vehicle mass and inertia tensor as propellant burns
% Linear depletion model: mass decreases linearly during burn, then holds constant

%% --- Editable parameters ---
m_dry    = 0.8;    % kg, mass with no propellant (structure + avionics + empty motors)
m_prop   = 0.05;   % kg, total propellant mass (KNSB)
t_burn   = 20;     % s, burn duration (must match ThrustMixer)

Ixx_dry  = 0.002;  % kg*m^2, roll-axis inertia (about x_b, nose axis), empty
Iyy_dry  = 0.05;   % kg*m^2, pitch-axis inertia (about y_b), empty
Izz_dry  = 0.05;   % kg*m^2, yaw-axis inertia (about z_b), empty

% How much extra inertia the propellant mass contributes when full
% (rough approximation: propellant sits near the tail, contributes mostly to pitch/yaw)
Ixx_prop = 0.0002;
Iyy_prop = 0.01;
Izz_prop = 0.01;

%% --- Mass as a function of time ---
if t < t_burn
    frac_burned = t / t_burn;      % 0 at ignition, 1 at burnout
else
    frac_burned = 1;
end

m_remaining_prop = m_prop * (1 - frac_burned);
m = m_dry + m_remaining_prop;

%% --- Inertia as a function of remaining propellant ---
Ixx = Ixx_dry + Ixx_prop * (1 - frac_burned);
Iyy = Iyy_dry + Iyy_prop * (1 - frac_burned);
Izz = Izz_dry + Izz_prop * (1 - frac_burned);

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

%% --- dI/dt (derivative of inertia w.r.t. time) ---
if t < t_burn
    dIxx_dt = -Ixx_prop / t_burn;
    dIyy_dt = -Iyy_prop / t_burn;
    dIzz_dt = -Izz_prop / t_burn;
else
    dIxx_dt = 0;
    dIyy_dt = 0;
    dIzz_dt = 0;
end

dIdt = [dIxx_dt 0 0; 0 dIyy_dt 0; 0 0 dIzz_dt];

end