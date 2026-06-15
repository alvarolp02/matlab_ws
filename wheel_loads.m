function F_z = wheel_loads(ax, ay, FL, FD)

%% Ensure column vectors

ax = ax(:);
ay = ay(:);
FL = FL(:);
FD = FD(:);

N = length(ax);

%% Vehicle parameters

g = 9.81;

% Geometry
l = 1.53;
b_v = 1.22;
b_h = 1.22;

balance = 0.441;

l_v = l*(1-balance);     % Front axle -> CG
l_h = l-l_v;             % Rear axle -> CG

% Masses
m = 170;

nsm_f = 21.9767;
nsm_r = 21.437436;

sm = m - nsm_f - nsm_r;

% Suspended mass distribution
l_v_sm = 0.80294677;
l_h_s  = l-l_v_sm;

sm_f = sm*l_h_s/l;
sm_r = sm*l_v_sm/l;

% Heights
h_COG_nsm_f = 0.20605;
h_COG_nsm_r = 0.20774991;

h_COG_sm = 0.30960693;

h_RC_f = 0.027;
h_RC_r = 0.075;

h_RA = h_RC_f + (h_RC_r-h_RC_f)*l_v/l;

% Anti geometry
brake_bal = 0.7;
p_th = 0.3;

z_IC_f = 0.127;
x_IC_f = -1.96783854166667;

z_IC_r = 0.12;
x_IC_r = 2.23125;

r_dyn = 0.20048;

AD_f = brake_bal*l/h_COG_sm*(z_IC_f-r_dyn)/(-x_IC_f);
AL_f = p_th*l/h_COG_sm*(z_IC_f-r_dyn)/(-x_IC_f);

AR_r = (1-brake_bal)*l/h_COG_sm*(z_IC_r-r_dyn)/(-x_IC_r);
AS_r = (1-p_th)*l/h_COG_sm*(z_IC_r-r_dyn)/(-x_IC_r);

%% Roll stiffness distribution

K_s_f   = 110000;
K_s_r   = 120000;

K_ARB_f = 5732;
K_ARB_r = 3356;

MR_s_f = 1.1993;
MR_s_r = 1.1992;

RS_f = (K_s_f*MR_s_f^2 + K_ARB_f)*b_v^2/2;
RS_r = (K_s_r*MR_s_r^2 + K_ARB_r)*b_h^2/2;

RS = RS_f + RS_r;

%% Lateral load transfer

y_WT_ns_f = nsm_f .* ay .* h_COG_nsm_f ./ b_v;
y_WT_ns_r = nsm_r .* ay .* h_COG_nsm_r ./ b_h;

y_WT_s_g_f = sm_f .* ay .* h_RC_f ./ b_v;
y_WT_s_g_r = sm_r .* ay .* h_RC_r ./ b_h;

y_WT_s_e_f = sm .* ay .* (h_COG_sm-h_RA) .* RS_f ./ RS ./ b_v;
y_WT_s_e_r = sm .* ay .* (h_COG_sm-h_RA) .* RS_r ./ RS ./ b_h;

%% Longitudinal load transfer

x_WT_ns = ...
    (nsm_f .* ax .* h_COG_nsm_f + ...
     nsm_r .* ax .* h_COG_nsm_r) ./ l;

x_WT_s = sm .* ax .* h_COG_sm ./ l;

x_WT_s_e_f = ...
    -x_WT_s .* (1-AD_f) .* (ax < 0) ...
    -x_WT_s .* (1-AL_f) .* (ax > 0);

x_WT_s_e_r = ...
     x_WT_s .* (1-AR_r) .* (ax < 0) ...
    +x_WT_s .* (1-AS_r) .* (ax > 0);

%% Static loads

Fz_front_static = 0.5*m*g*l_h/l;
Fz_rear_static  = 0.5*m*g*l_v/l;

%% Aero distribution

Fz_aero_front = ...
      0.5*(l-l_v)/l .* FL ...
    - 0.5*h_RA/l .* FD;

Fz_aero_rear = ...
      0.5*l_v/l .* FL ...
    + 0.5*h_RA/l .* FD;

%% Wheel loads

Fz_FL = ...
      Fz_front_static ...
    + Fz_aero_front ...
    - y_WT_ns_f ...
    - y_WT_s_g_f ...
    - y_WT_s_e_f ...
    - 0.5*(x_WT_ns+x_WT_s);

Fz_FR = ...
      Fz_front_static ...
    + Fz_aero_front ...
    + y_WT_ns_f ...
    + y_WT_s_g_f ...
    + y_WT_s_e_f ...
    - 0.5*(x_WT_ns+x_WT_s);

Fz_RL = ...
      Fz_rear_static ...
    + Fz_aero_rear ...
    - y_WT_ns_r ...
    - y_WT_s_g_r ...
    - y_WT_s_e_r ...
    + 0.5*(x_WT_ns+x_WT_s);

Fz_RR = ...
      Fz_rear_static ...
    + Fz_aero_rear ...
    + y_WT_ns_r ...
    + y_WT_s_g_r ...
    + y_WT_s_e_r ...
    + 0.5*(x_WT_ns+x_WT_s);

%% Assemble output matrix

F_z = [Fz_FL, Fz_FR, Fz_RL, Fz_RR];

%% Avoid negative loads

F_z(F_z < 0) = 0;

end