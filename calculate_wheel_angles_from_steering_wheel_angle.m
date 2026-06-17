function [delta_fl, delta_fr] = calculate_wheel_angles_from_steering_wheel_angle(steeringAngleRad)

%% Parámetros del vehículo (VehicleParams)

static_toe_f = -0.5;   % [deg]

% Steering
i_hw_rack             = 0.3089;
steer_coef0           = 0.112976705633141;
steer_coef1           = 0.921364395821404;
steer_coef2           = -0.00481375121199615;
steering_transmission = 3.6364;

%% Conversión volante -> rueda

% Ángulo en el volante [rad]
steeringAngleAtSteeringWheelRad = ...
    steering_transmission * steeringAngleRad;

% Convertir a grados
steeringAngleAtSteeringWheelDEG = ...
    steeringAngleAtSteeringWheelRad * 180/pi;

% Desplazamiento de cremallera
deltaRack = steeringAngleAtSteeringWheelDEG * i_hw_rack;

%% Polinomio de Ackermann

ackermannPoly = @(x) ...
    steer_coef2 .* x.^2 + ...
    steer_coef1 .* x + ...
    steer_coef0;

%% Ángulos de rueda [deg]

delta_fl_deg = -ackermannPoly(-deltaRack) + static_toe_f;
delta_fr_deg =  ackermannPoly( deltaRack) - static_toe_f;

%% Convertir a radianes

delta_fl = delta_fl_deg * pi/180;
delta_fr = delta_fr_deg * pi/180;

end