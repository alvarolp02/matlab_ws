% DEPENDENCIES FOR BUILDING ROS2 MSGS:
% MATLAB ROS Toolbox
% CMake (Windows installer)
% C++ compiler (Microsoft Visual C++ 2022) (mex -setup cpp)
% python 3.10 (setup also in ROS Toolbox preferences)
customMsgFolder = 'custom_msgs';   % contains interfaces/msg and shared_interfaces/msg
ros2genmsg(customMsgFolder)

%% Open bag
bag = ros2bagreader('bags/recovered_mcu_bag_06-16-2026_21-47_0.mcap');

%% Select and extract all desired topics
% topicNames = bag.AvailableTopics.Row;
topicNames = {'/gss_data', '/dv/imu', '/dv/steering_angle', '/dv/actual_torque', '/dv/wheel_speed'};

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

%% Synchronize all data to imu msgs
% Reference timestamps from imu
t_ref = data.dv_imu.timestamps;

% Helper: interpolate any signal to t_ref
sync = @(t, v) interp1(t, v, t_ref, 'linear', 'extrap');

% dv/imu  (sensor_msgs/Imu)
imu_ax = cellfun(@(m) m.linear_acceleration.x, data.dv_imu.msgs);
imu_ay = cellfun(@(m) m.linear_acceleration.y, data.dv_imu.msgs);
imu_r = cellfun(@(m) m.angular_velocity.z,    data.dv_imu.msgs);
r = imu_r;

% dv/steering_angle  (std_msgs/Float64 → .data)
t_steer = data.dv_steering_angle.timestamps;
steer = cellfun(@(m) m.data, data.dv_steering_angle.msgs);
steer   = sync(t_steer, steer);

% gss_data  (sensor_msgs/Imu)
t_gss = data.gss_data.timestamps;
vx = cellfun(@(m) m.v_x, data.gss_data.msgs)/3.6; %kmh to ms
vy = cellfun(@(m) m.v_y, data.gss_data.msgs); %kmh to ms
vx = sync(t_gss, vx);
vy = sync(t_gss, vy);

% dv/actual_torque  (shared_interfaces/WheelData)
t_torque  = data.dv_actual_torque.timestamps;
torque_fl = cellfun(@(m) m.front_left,  data.dv_actual_torque.msgs);
torque_fr = cellfun(@(m) m.front_right, data.dv_actual_torque.msgs);
torque_rl = cellfun(@(m) m.rear_left,   data.dv_actual_torque.msgs);
torque_rr = cellfun(@(m) m.rear_right,  data.dv_actual_torque.msgs);
torque_fl = sync(t_torque, torque_fl);
torque_fr = sync(t_torque, torque_fr);
torque_rl = sync(t_torque, torque_rl);
torque_rr = sync(t_torque, torque_rr);

% dv/wheel_speed (shared_interfaces/WheelData)
t_wenc    = data.dv_wheel_speed.timestamps;
wenc_fl = cellfun(@(m) m.front_left,  data.dv_wheel_speed.msgs);
wenc_fr = cellfun(@(m) m.front_right,  data.dv_wheel_speed.msgs);
wenc_rl = cellfun(@(m) m.rear_left,  data.dv_wheel_speed.msgs);
wenc_rr = cellfun(@(m) m.rear_right,  data.dv_wheel_speed.msgs);
wenc_fl = sync(t_wenc, wenc_fl)*2*pi/60; % rpm to rad/s
wenc_fr = sync(t_wenc, wenc_fr)*2*pi/60; % rpm to rad/s
wenc_rl = sync(t_wenc, wenc_rl)*2*pi/60; % rpm to rad/s
wenc_rr = sync(t_wenc, wenc_rr)*2*pi/60; % rpm to rad/s

% All variables now have the same size as vehicle_state
fprintf('All signals synchronized: %d samples\n', numel(t_ref));