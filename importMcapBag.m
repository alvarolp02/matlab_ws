% DEPENDENCIES FOR BUILDING ROS2 MSGS:
% MATLAB ROS Toolbox
% CMake (Windows installer)
% C++ compiler (Microsoft Visual C++ 2022) (mex -setup cpp)
% python 3.10 (setup also in ROS Toolbox preferences)
customMsgFolder = 'C:/Users/alple/Documents/matlab_ws/custom_msgs';   % contains interfaces/msg and shared_interfaces/msg
ros2genmsg(customMsgFolder)

%% Open bag
bag = ros2bagreader('bags/trackdrive_2026-06-06_19-27-14_compressed.mcap');

%% Select and extract all desired topics
% topicNames = bag.AvailableTopics.Row;
topicNames = {'/controller/steering_request', '/sensing/steering_angle', '/controller/torque_request', '/sensing/gss', '/slam/vehicle_state', '/sensing/imu', '/controller/wheel_slip_angles', '/estimation/debug/slip_ratio', '/sensing/wheel_encoder'};

sel = select(bag, 'Topic', topicNames);

allMsgs = readMessages(sel);
allTimes = sel.MessageList.Time;
allTopics = sel.MessageList.Topic;

% Split by topic and populate struct
t0 = allTimes(1);
data = struct();
for i = 1:numel(topicNames)
    topicName = topicNames{i};
    fieldName = regexprep(topicName, '^/', '');
    fieldName = strrep(fieldName, '/', '_');

    % Handle both string array and cell array
    if iscell(allTopics)
        idx = strcmp(allTopics, topicName);
    else
        idx = allTopics == string(topicName);
    end

    idx = logical(idx);
    nMsgs = sum(idx);

    if nMsgs > 0
        data.(fieldName).msgs = allMsgs(idx);
        data.(fieldName).timestamps = double(allTimes(idx)) - t0;
    end

    fprintf('Loaded: %s (%d messages)\n', topicName, nMsgs);
end

%% Overview plots
x =  cellfun(@(m) m.x, data.slam_vehicle_state.msgs);
y =  cellfun(@(m) m.y, data.slam_vehicle_state.msgs);
vx = cellfun(@(m) m.vx, data.slam_vehicle_state.msgs);
axis equal

plot3(x,y,vx)

%% Synchronize all data to vehicle_state msgs
% Reference timestamps from vehicle_state
t_ref = data.slam_vehicle_state.timestamps;

% Helper: interpolate any signal to t_ref
sync = @(t, v) interp1(t, v, t_ref, 'linear', 'extrap');

% slam/vehicle_state (reference, no interpolation needed)
vx      = cellfun(@(m) m.vx,  data.slam_vehicle_state.msgs);
vy      = cellfun(@(m) m.vy,  data.slam_vehicle_state.msgs);
r       = cellfun(@(m) m.r,   data.slam_vehicle_state.msgs);
x       = cellfun(@(m) m.x,   data.slam_vehicle_state.msgs);
y       = cellfun(@(m) m.y,   data.slam_vehicle_state.msgs);
yaw     = cellfun(@(m) m.yaw, data.slam_vehicle_state.msgs);

% sensing/steering_angle  (std_msgs/Float64 → .data)
t_steer = data.sensing_steering_angle.timestamps;
steer = cellfun(@(m) m.data, data.sensing_steering_angle.msgs);
steer   = sync(t_steer, steer);

% sensing/gss  (shared_interfaces/GSSData)
t_gss   = data.sensing_gss.timestamps;
gss_vx  = cellfun(@(m) m.v_x, data.sensing_gss.msgs);
gss_vy  = cellfun(@(m) m.v_y, data.sensing_gss.msgs);
gss_vx  = sync(t_gss, gss_vx);
gss_vy  = sync(t_gss, gss_vy);

% sensing/imu  (sensor_msgs/Imu)
t_imu = data.sensing_imu.timestamps;
imu_ax = cellfun(@(m) m.linear_acceleration.x, data.sensing_imu.msgs);
imu_ay = cellfun(@(m) m.linear_acceleration.y, data.sensing_imu.msgs);
imu_r = cellfun(@(m) m.angular_velocity.z,    data.sensing_imu.msgs);
imu_ax = sync(t_imu, imu_ax);
imu_ay = sync(t_imu, imu_ay);
imu_r = sync(t_imu, imu_r);

% controller/steering_request  (std_msgs/Float64)
t_steer_req   = data.controller_steering_request.timestamps;
steer_req = cellfun(@(m) m.data, data.controller_steering_request.msgs);
steer_req = sync(t_steer_req, steer_req);

% controller/torque_request  (shared_interfaces/WheelData)
t_torque  = data.controller_torque_request.timestamps;
torque_fl = cellfun(@(m) m.front_left,  data.controller_torque_request.msgs);
torque_fr = cellfun(@(m) m.front_right, data.controller_torque_request.msgs);
torque_rl = cellfun(@(m) m.rear_left,   data.controller_torque_request.msgs);
torque_rr = cellfun(@(m) m.rear_right,  data.controller_torque_request.msgs);
torque_fl = sync(t_torque, torque_fl);
torque_fr = sync(t_torque, torque_fr);
torque_rl = sync(t_torque, torque_rl);
torque_rr = sync(t_torque, torque_rr);

% controller/wheel_slip_angles  (shared_interfaces/WheelData)
t_slip    = data.controller_wheel_slip_angles.timestamps;
slip_fl   = cellfun(@(m) m.front_left,  data.controller_wheel_slip_angles.msgs);
slip_fr   = cellfun(@(m) m.front_right, data.controller_wheel_slip_angles.msgs);
slip_rl   = cellfun(@(m) m.rear_left,   data.controller_wheel_slip_angles.msgs);
slip_rr   = cellfun(@(m) m.rear_right,  data.controller_wheel_slip_angles.msgs);
slip_fl   = sync(t_slip, slip_fl);
slip_fr   = sync(t_slip, slip_fr);
slip_rl   = sync(t_slip, slip_rl);
slip_rr   = sync(t_slip, slip_rr);

% estimation/debug/slip_ratio  (shared_interfaces/WheelData)
t_slipr    = data.estimation_debug_slip_ratio.timestamps;
slipr_fl   = cellfun(@(m) m.front_left,  data.estimation_debug_slip_ratio.msgs);
slipr_fr   = cellfun(@(m) m.front_right, data.estimation_debug_slip_ratio.msgs);
slipr_rl   = cellfun(@(m) m.rear_left,   data.estimation_debug_slip_ratio.msgs);
slipr_rr   = cellfun(@(m) m.rear_right,  data.estimation_debug_slip_ratio.msgs);
slipr_fl   = sync(t_slipr, slipr_fl);
slipr_fr   = sync(t_slipr, slipr_fr);
slipr_rl   = sync(t_slipr, slipr_rl);
slipr_rr   = sync(t_slipr, slipr_rr);

% sensing/wheel_encoder (shared_interfaces/WheelData)
t_w    = data.sensing_wheel_encoder.timestamps;
w_fl = cellfun(@(m) m.front_left,  data.sensing_wheel_encoder.msgs);
w_fr = cellfun(@(m) m.front_right,  data.sensing_wheel_encoder.msgs);
w_rl = cellfun(@(m) m.rear_left,  data.sensing_wheel_encoder.msgs);
w_rr = cellfun(@(m) m.rear_right,  data.sensing_wheel_encoder.msgs);
w_fl = sync(t_w, w_fl)*2*pi/60; % rpm to rad/s
w_fr = sync(t_w, w_fr)*2*pi/60; % rpm to rad/s
w_rl = sync(t_w, w_rl)*2*pi/60; % rpm to rad/s
w_rr = sync(t_w, w_rr)*2*pi/60; % rpm to rad/s

% All variables now have the same size as vehicle_state
fprintf('All signals synchronized: %d samples\n', numel(t_ref));

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
    'FL',        lf,   tw/2,   true,     1,      w_fl,   torque_fl,  1;
    'FR',        lf,  -tw/2,   true,     2,      w_fr,   torque_fr,  2;
    'RL',       -lr,   tw/2,   false,    3,      w_rl,   torque_rl,  3;
    'RR',       -lr,  -tw/2,   false,    4,      w_rr,   torque_rr,  4;
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

