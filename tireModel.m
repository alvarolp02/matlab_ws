% MAKE SURE TO HAVE vx, vy, r, x, y, yaw, imu_ax, imu_ay, imu_r, steer, wencs AND torques VECTORS WITH THE SAME SIZE 

%% Tire model

% Params
tw = 1.22; % TrackWidth
wb = 1.53; % Wheelbase
cogRatioFront = 0.441;
lf = wb*(1-cogRatioFront); % Dist from cog to front axle
lr = wb - lf; % Dist from cog to rear axle
Rw = 0.20048; % Wheel radius
Gr = 12.009; % Gear ratio
Iw = 0.580; % Moment of inertia of wheels respect to the axle
m = 173.7; % Mass
Iz = 125.39603; % Rotational Inertia around z axis
cl = 5.0;
cd = 1.9;
A = 1;
ro = 1.2;

% Tire parameters per wheel
% [name, lx_sign, ly_sign, is_front, Fy_col]
% lyi = +tw/2 (left) or -tw/2 (right)
% lxi = +lf (front) or -lr (rear)
wheels = {
    %label      lxi    lyi   is_front  Fy_col wheel_encoder torque Fz_col
    'FL',        lf,   tw/2,   true,     1,      wenc_fl,   torque_fl,  1;
    'FR',        lf,  -tw/2,   true,     2,      wenc_fr,   torque_fr,  2;
    'RL',       -lr,   tw/2,   false,    3,      wenc_rl,   torque_rl,  3;
    'RR',       -lr,  -tw/2,   false,    4,      wenc_rr,   torque_rr,  4;
    };

% Precompute shared signals
v_abs = sqrt(vx.^2 + vy.^2);
FL_aero = 0.5*m*A*cl*ro*(v_abs.^2);
FD_aero = 0.5*m*A*cd*ro*(v_abs.^2);
Fz_all  = wheel_loads(imu_ax, imu_ay, FL_aero, FD_aero);  % Nx4
Fz_fl = Fz_all(:,1);
Fz_fr = Fz_all(:,2);
Fz_rl = Fz_all(:,3);
Fz_rr = Fz_all(:,4);
Fz_f = Fz_fl + Fz_fr;
Fz_r = Fz_rl + Fz_rr;

r_p  = gradient(r);
vy_p = gradient(vy);
Fyf  = (m.*(vy_p + vx.*r)*lr + Iz.*r_p) / (lf + lr);
Fyr  = (m.*(vy_p + vx.*r)*lf - Iz.*r_p) / (lf + lr);
% Distribute lateral forces proportional to wheel load distr on each axis
Fy_per_wheel = [Fyf.*(Fz_fl./Fz_f), Fyf.*(Fz_fr./Fz_f), Fyr.*(Fz_rl./Fz_r), Fyr.*(Fz_rr./Fz_r)];

% Loop over wheels
figure; tiledlayout(2,2);
ax_kFx = gobjects(4,1);
for i = 1:size(wheels,1)
    ax_kFx(i) = nexttile;
end

figure; tiledlayout(2,2);
ax_aFy = gobjects(4,1);
for i = 1:size(wheels,1)
    ax_aFy(i) = nexttile;
end

for i = 1:size(wheels, 1)
    label    = wheels{i,1};
    lxi      = wheels{i,2};
    lyi      = wheels{i,3};
    is_front = wheels{i,4};
    fy_col   = wheels{i,5};
    wi       = wheels{i,6};
    Ti       = wheels{i,7};
    fz_col   = wheels{i,8};

    % Speed at wheel center
    vxi = vx - r.*lyi;
    vyi = vy + r.*lxi;

    % Rotate to wheel frame
    if is_front
        vx_wheel_i =  vxi.*cos(steer) + vyi.*sin(steer);
        vy_wheel_i = -vxi.*sin(steer) + vyi.*cos(steer);
    else
        vx_wheel_i = vxi;
        vy_wheel_i = vyi;
    end

    % Regularization
    wi         = wi         + 1e-9;
    vx_wheel_i = vx_wheel_i + 1e-9;

    % Slip ratio
    k_i = (wi.*Rw./Gr - vx_wheel_i) ./ max(abs(wi.*Rw./Gr), abs(vx_wheel_i));

    % Slip angle
    a_i = -atan(vy_wheel_i ./ vx_wheel_i);

    % Longitudinal force
    wi_p = gradient(wi);
    Fx_i = (Ti - Iw.*wi_p) ./ Rw;

    % Lateral force
    Fy_i = Fy_per_wheel(:, fy_col);

    % Wheel load
    Fz_i = Fz_all(:, fz_col);

    % --- Fx vs slip ratio ---
    axes(ax_kFx(i));
    scatter(k_i, Fx_i, 4, '.'); grid on;
    xlabel('Slip Ratio \kappa'); ylabel('F_x [N]');
    title(sprintf('Longitudinal - %s', label));

    % --- Fy vs slip angle ---
    axes(ax_aFy(i));
    scatter(rad2deg(a_i), Fy_i, 4, '.'); grid on;
    xlabel('Slip Angle [deg]'); ylabel('F_y [N]');
    title(sprintf('Lateral - %s', label));
end

% Add figure titles
figure(1); sgtitle('Longitudinal Tire Behaviour');
figure(2); sgtitle('Lateral Tire Behaviour');

