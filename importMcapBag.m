% DEPENDENCIES FOR BUILDING ROS2 MSGS:
% MATLAB ROS Toolbox
% CMake (Windows installer)
% C++ compiler (Microsoft Visual C++ 2022) (mex -setup cpp)
% python 3.10 (setup also in ROS Toolbox preferences)
customMsgFolder = 'C:/Users/alple/Documents/matlab_ws/custom_msgs';   % contains interfaces/msg and shared_interfaces/msg
ros2genmsg(customMsgFolder)

%% Open bag
bag = ros2bagreader('bags/trackdrive_2026-06-06_19-03-12_compressed.mcap');

%% Select and extract all desired topics
% topicNames = bag.AvailableTopics.Row;
topicNames = {'/controller/steering_request', '/sensing/steering_angle', '/controller/torque_request', '/sensing/gss', '/slam/vehicle_state', '/sensing/imu', '/controller/wheel_slip_angles', '/estimation/debug/slip_ratio', '/sensing/wheel_encoder'};

sel = select(bag, 'Topic', topicNames);

allMsgs = readMessages(sel);
allTimes = sel.MessageList.Time;
allTopics = sel.MessageList.Topic;

% Split by topic and populate struct
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
        data.(fieldName).timestamps = double(allTimes(idx)) - double(allTimes(find(idx, 1)));
    end

    fprintf('Loaded: %s (%d messages)\n', topicName, nMsgs);
end

%% Overview plots
x =  cellfun(@(m) m.x, data.slam_vehicle_state.msgs);
y =  cellfun(@(m) m.y, data.slam_vehicle_state.msgs);
vx = cellfun(@(m) m.vx, data.slam_vehicle_state.msgs);

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
w_fl = sync(t_w, w_fl);
w_fr = sync(t_w, w_fr);
w_rl = sync(t_w, w_rl);
w_rr = sync(t_w, w_rr);

% All variables now have the same size as vehicle_state
fprintf('All signals synchronized: %d samples\n', numel(t_ref));

%% Tire model

tw = 1.22; % TrackWidth
wb = 1.53; % Wheelbase
cogRatioFront = 0.441;
lf = wb*cogRatioFront; % Dist from cog to front axle
Rw = 0.20048; % Wheel radius

% Choose front left tire
lyi = tw/2;
lxi = lf;
wi = w_fl;

% Speed in the center of the wheel
vxi = vx - r.*lyi;
vyi = vy + r.*lxi;

% Longitudinal speed on the wheel (front wheels)
vx_wheel_i = vxi.*cos(steer) + vyi.*sin(steer); % Assuming steering_angle = steer of the individual wheel
vy_wheel_i = -vxi.*sin(steer) + vyi.*cos(steer);
%vx_wheel_i = vxi , vy_wheel_i = vyi % (rear wheels)

% Slip Ratio calculation
ki = (wi.*Rw - vx_wheel_i)./max(abs(wi.*Rw), abs(vx_wheel_i));