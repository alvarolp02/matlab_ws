% DEPENDENCIES FOR BUILDING ROS2 MSGS:
% MATLAB ROS Toolbox
% CMake (Windows installer)
% C++ compiler (Microsoft Visual C++ 2022) (mex -setup cpp)
% python 3.10 (setup also in ROS Toolbox preferences)
customMsgFolder = 'custom_msgs';   % contains interfaces/msg and shared_interfaces/msg
ros2genmsg(customMsgFolder)

%% Open bag
bag = ros2bagreader('bags/trackdrive_2026-06-06_19-03-12_compressed.mcap');

%% Select and extract all desired topics
% topicNames = bag.AvailableTopics.Row;
topicNames = {'/controller/steering_request', '/sensing/actual_torque', '/sensing/steering_angle', '/controller/torque_request', '/sensing/gss', '/slam/vehicle_state', '/sensing/imu', '/controller/wheel_slip_angles', '/estimation/debug/slip_ratio', '/sensing/wheel_encoder'};

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
% vx      = cellfun(@(m) m.vx,  data.slam_vehicle_state.msgs);
% vy      = cellfun(@(m) m.vy,  data.slam_vehicle_state.msgs);
r       = cellfun(@(m) m.r,   data.slam_vehicle_state.msgs);
x       = cellfun(@(m) m.x,   data.slam_vehicle_state.msgs);
y       = cellfun(@(m) m.y,   data.slam_vehicle_state.msgs);
yaw     = cellfun(@(m) m.yaw, data.slam_vehicle_state.msgs);


t_gss = data.sensing_gss.timestamps;
vx = cellfun(@(m) m.v_x, data.sensing_gss.msgs);
vy = cellfun(@(m) m.v_y, data.sensing_gss.msgs);
vx = sync(t_gss,vx);
vy = sync(t_gss,vy);
delta = -0.06;
vxR = vx*cos(delta)-vy*sin(delta);
vyR = vx*sin(delta)+vy*cos(delta);
vx = vxR;
vy = vyR;

vy = vy - (lf+0.127)*r; % ggs 

% sensing/steering_angle  (std_msgs/Float64 → .data)
t_steer = data.sensing_steering_angle.timestamps;
steer = cellfun(@(m) m.data, data.sensing_steering_angle.msgs);
steer   = sync(t_steer, steer);

t_steer = data.controller_steering_request.timestamps;
steerC = cellfun(@(m) m.data, data.controller_steering_request.msgs);
steerC   = sync(t_steer, steerC);

% sensing/imu  (sensor_msgs/Imu)
t_imu = data.sensing_imu.timestamps;
imu_ax = cellfun(@(m) m.linear_acceleration.x, data.sensing_imu.msgs);
imu_ay = cellfun(@(m) m.linear_acceleration.y, data.sensing_imu.msgs);
imu_r = cellfun(@(m) m.angular_velocity.z,    data.sensing_imu.msgs);
imu_ax = sync(t_imu, imu_ax);
imu_ay = sync(t_imu, imu_ay);
imu_r = sync(t_imu, imu_r);
delta2 = deg2rad(-3.5);
axR = imu_ax*cos(delta2)-imu_ay*sin(delta2);
ayR = imu_ax*sin(delta2)+imu_ay*cos(delta2);
imu_ax = axR;
imu_ay = ayR;

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

% sensing/wheel_encoder (shared_interfaces/WheelData)
t_wenc    = data.sensing_wheel_encoder.timestamps;
wenc_fl = cellfun(@(m) m.front_left,  data.sensing_wheel_encoder.msgs);
wenc_fr = cellfun(@(m) m.front_right,  data.sensing_wheel_encoder.msgs);
wenc_rl = cellfun(@(m) m.rear_left,  data.sensing_wheel_encoder.msgs);
wenc_rr = cellfun(@(m) m.rear_right,  data.sensing_wheel_encoder.msgs);
wenc_fl = sync(t_wenc, wenc_fl)*2*pi/60; % rpm to rad/s
wenc_fr = sync(t_wenc, wenc_fr)*2*pi/60; % rpm to rad/s
wenc_rl = sync(t_wenc, wenc_rl)*2*pi/60; % rpm to rad/s
wenc_rr = sync(t_wenc, wenc_rr)*2*pi/60; % rpm to rad/s

% All variables now have the same size as vehicle_state
fprintf('All signals synchronized: %d samples\n', numel(t_ref));