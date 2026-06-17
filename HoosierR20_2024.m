function Pac = HoosierR20_2024()

%% ============================================================
% Hoosier R20 State 2024
% Pacejka parameters
% ============================================================

%% Physical characteristics
Pac.F_z0_pac = 700;
Pac.R_0      = 0.207;
Pac.r_dyn    = 0.20048;

%% Scaling factors
Pac.LFZ0  = 1;
Pac.F_z0  = Pac.F_z0_pac * Pac.LFZ0;

Pac.LCX   = 1;
Pac.LMUX  = 0.85;
Pac.LEX   = 1;
Pac.LKX   = 1;
Pac.LHX   = 1;
Pac.LVX   = 1;
Pac.LGAX  = 1;

Pac.LCY   = 1;
Pac.LMUY  = 0.5561;
% Pac.LMUY = 0.85;
Pac.LEY   = 1;
Pac.LKY   = 0.9;
Pac.LHY   = 1;
Pac.LVY   = 1;
Pac.LGAY  = 1;

Pac.LTR   = 1;
Pac.LGAZ  = 1;
Pac.LXAL  = 1;
Pac.LYKA  = 1;
Pac.LVYKA = 1;
Pac.LS    = 1;
Pac.LSGKP = 1;
Pac.LSGAL = 1;
Pac.LGYR  = 1;
Pac.LMX   = 1;
Pac.LVMX  = 1;
Pac.LMY   = 1;
Pac.LRES  = 1;

%% ============================================================
% Longitudinal force (pure slip)
%% ============================================================

Pac.p_Cx1 = 1.5125;

Pac.p_Dx1 = 1.99;
Pac.p_Dx2 = -0.0098;
Pac.p_Dx3 = 9.9959;

Pac.p_Ex1 = -0.15;
Pac.p_Ex2 = 0.30002;
Pac.p_Ex3 = 0.00004;
Pac.p_Ex4 = 0;

Pac.p_Kx1 = 51.5;
Pac.p_Kx2 = 10.007;
Pac.p_Kx3 = 0.3021;

Pac.p_Hx1 = 0;
Pac.p_Hx2 = 0;

Pac.p_Vx1 = 0;
Pac.p_Vx2 = 0;

%% ============================================================
% Longitudinal force (combined slip braking)
%% ============================================================

Pac.r_Bx1b = 12;
Pac.r_Bx2b = 11.5041;
Pac.r_Cx1b = 0.95;
Pac.r_Ex1b = -3.7106;
Pac.r_Ex2b = -4.3911;
Pac.r_Hx1b = 0;

%% ============================================================
% Longitudinal force (combined slip acceleration)
%% ============================================================

Pac.r_Bx1 = 12;
Pac.r_Bx2 = 11.5041;
Pac.r_Cx1 = 0.95;
Pac.r_Ex1 = -3.7106;
Pac.r_Ex2 = -4.3911;
Pac.r_Hx1 = 0;

%% ============================================================
% Lateral force (pure slip)
%% ============================================================

Pac.p_Cy1 = 1.5;

Pac.p_Dy1 = 1.86;
Pac.p_Dy2 = -0.0285;
Pac.p_Dy3 = -100;

Pac.p_Ey1 = -0.6950;
Pac.p_Ey2 = -0.25;
Pac.p_Ey3 = 0;
Pac.p_Ey4 = -0.65;

Pac.p_Ky1 = -64.5;
Pac.p_Ky2 = 2.05;
Pac.p_Ky3 = 0.55813;

Pac.p_Hy1 = 0;
Pac.p_Hy2 = 0;
Pac.p_Hy3 = -0.1;

Pac.p_Vy1 = 0;
Pac.p_Vy2 = 0;
Pac.p_Vy3 = -1.7505;
Pac.p_Vy4 = 1.9117;

%% ============================================================
% Lateral force (combined slip)
%% ============================================================

Pac.r_By1 = 16.1062;
Pac.r_By2 = 28.75;
Pac.r_By3 = 0.10646;

Pac.r_Cy1 = 0.932;

Pac.r_Ey1 = -0.59944;
Pac.r_Ey2 = -0.95;

Pac.r_Hy1 = 0;
Pac.r_Hy2 = 0;

Pac.r_Vy1 = 0;
Pac.r_Vy2 = 0;
Pac.r_Vy3 = 0;
Pac.r_Vy4 = 0;
Pac.r_Vy5 = 0;
Pac.r_Vy6 = 0;

Pac.p_Ty1 = 0;
Pac.p_Ty2 = 0;

%% ============================================================
% Aligning torque
%% ============================================================

Pac.q_Bz1  = 9.560;
Pac.q_Bz2  = -6.547;
Pac.q_Bz3  = -3;

Pac.q_Bz4  = -42.38;
Pac.q_Bz5  = 42.44;

Pac.q_Bz9  = 7.203;
Pac.q_Bz10 = 1.99;

Pac.q_Cz1 = 1.245;

Pac.q_Dz1 = 0.11;
Pac.q_Dz2 = 0.0485;
Pac.q_Dz3 = 0.2903;
Pac.q_Dz4 = -15.3;

Pac.q_Dz6 = -0.000264;
Pac.q_Dz7 = -0.003341;
Pac.q_Dz8 = 1.411;
Pac.q_Dz9 = -0.0821;

Pac.q_Ez1 = -0.391;
Pac.q_Ez2 = -2.9;
Pac.q_Ez3 = -2.55;
Pac.q_Ez4 = 0.274;
Pac.q_Ez5 = -6.954;

Pac.q_Hz1 = 0;
Pac.q_Hz2 = 0;
Pac.q_Hz3 = 0;
Pac.q_Hz4 = 0;

Pac.s_sz1 = -0.095;
Pac.s_sz2 = -0.0328;
Pac.s_sz3 = 0;
Pac.s_sz4 = 0;

%% ============================================================
% Rolling resistance
%% ============================================================

Pac.q_Sy1 = -3.090e-2;

%% ============================================================
% Derived constants
%% ============================================================

Pac.C_x   = Pac.p_Cx1 * Pac.LCX;
Pac.S_Hxa = Pac.r_Hx1;
Pac.C_xa  = Pac.r_Cx1;

%% ============================================================
% Pressure sensitivity parameters
%% ============================================================

Pac.p_0   = 83000;

Pac.p_Px1 = -0.9583;
Pac.p_Px2 = 0.3044;
Pac.p_Px3 = -0.0344;
Pac.p_Px4 = -0.1;

Pac.r_Bx3 = -247.5757;

%% Tire pressure
Pac.pressure = 10; % psi

Pac.dpi = (Pac.pressure * 6894.76 - Pac.p_0) / Pac.p_0;

end