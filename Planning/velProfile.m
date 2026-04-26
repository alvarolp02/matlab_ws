function [trajectory] = velProfile(racingLine)
%VEL_PROFILE Given a [x,y] racingLine, compute the velocity profile
% given ggv, mu, and max accelerations
% trajectory: [x,y,psi,s,vel,acc,k]

N = size(racingLine,1);

x = racingLine(:,1);
y = racingLine(:,2);

dx=gradient(x);
dy=gradient(y);
psi = atan2(dy, dx);

diffs = diff(racingLine(:,1:2),1,1);
segmentLengths = [0;hypot(diffs(:,1), diffs(:,2))];
s = cumsum(segmentLengths,1);

% Curvature: (dx*ddy - dy*ddx)/(dx^2+dy^2)^(3/2)
ddx=gradient(dx);
ddy=gradient(dy);
k=(dx.*ddy - dy.*ddx)./((dx.^2+dy.^2).^(3/2));

cL = 4.0;
cD = 1.93;
ro = 1.2;
g = 9.81;
A = 1;
m = 170.0;
eta = 0.94 * 0.94 * 0.96;
maxAccFactorController = 0.8;
P_thr = 80.0;
P_br = 60.0;
ax_th_scale_factor = 0.55; % scaling factor for throttle acceleration
ax_br_scale_factor = 1.0;  % scaling factor for braking acceleration



muX = 1.3291*0.6;
muY = 1.5863*0.55;

vMax = 25.0;

vGrip = zeros(N,1);
for i=1:N
    a=0.54728*(1/2)*cL*ro*A*(1/m)*muY - abs(k(i));
    b = 0.03825*cL;
    c = 0.95833*muY*g;
    v1 = (-b + sqrt(b^2 - 4*a*c))/(2*a);
    v2 = (-b - sqrt(b^2 - 4*a*c))/(2*a);

    if isnan(v1)
        v1 = sqrt(muY*g*0.95833/abs(k(i)));
    elseif isnan(v2)
        v2 = sqrt(muY*g*0.95833/abs(k(i)));
    end

    if v1<0 && v2<0
        vGrip(i) = sqrt(muY*g*0.95833/abs(k(i)));
    elseif v1>=0 && v2>=0
        vGrip(i) = min(v1, v2);
    elseif v1>0 && v2<0
        vGrip(i) = v1;
    else
        vGrip(i) = v2;
    end

end



vel = zeros(N,1);
vel(1) = vMax;
ds = segmentLengths;

% Forwards loop. First iteration. Calculate speed profile and limit by grip
for i = 2:N
    % Compute ax given curvature and velocity at this point
    ay=k(i-1)*vel(i-1)^2;
    ayMax = 0.54728*(1/2)*cL*ro*A*(1/m)*muY*(vel(i-1)^2) + 0.038*cL*vel(i-1)+0.958*muY*g;
    axMaxGGV = 0;
    if abs(ay)<ayMax
        axMax = 1.05*muX*g + 0.5*(1/m)*ro*A*(muX*cL-cD)*1.51*(vel(i-1)^2);
        axMaxGGV = axMax*sqrt(1-(ay/ayMax)^2);
    end
    axMaxController = maxAccFactorController*(g + 0.5*ro*A*cL*(vel(i-1)^2))/m;
    axMaxMotors = ax_th_scale_factor*((1/vel(i-1))*(P_thr*1000*eta/m) - 0.5*(ro*A/m)*(cD*vel(i-1)^2));
    ax = min(min(axMaxGGV, axMaxMotors),axMaxController);

    vel(i) = min(sqrt(vel(i-1)^2 + 2*ax*ds(i)), vGrip(i));
end

% Close loop
vel(1) = vel(m);

% Forwards loop. Second iteration
for i = 2:N
    % Compute ax given curvature and velocity at this point
    ay=k(i-1)*vel(i-1)^2;
    ayMax = 0.54728*(1/2)*cL*ro*A*(1/m)*muY*(vel(i-1)^2) + 0.038*cL*vel(i-1)+0.958*muY*g;
    axMaxGGV = 0;
    if abs(ay)<ayMax
        axMax = 1.05*muX*g + 0.5*(1/m)*ro*A*(muX*cL-cD)*1.51*(vel(i-1)^2);
        axMaxGGV = axMax*sqrt(1-(ay/ayMax)^2);
    end
    axMaxController = maxAccFactorController*(g + 0.5*ro*A*cL*(vel(i-1)^2))/m;
    axMaxMotors = ax_th_scale_factor*((1/vel(i-1))*(P_thr*1000*eta/m) - 0.5*(ro*A/m)*(cD*vel(i-1)^2));
    ax = min(min(axMaxGGV, axMaxMotors),axMaxController);

    vel(i) = min(sqrt(vel(i-1)^2 + 2*ax*ds(i)), vGrip(i));
end

% Backwards loop. First iteration. Limit speed by max braking
for j = N:-1:2
    % Compute ax given curvature and velocity at this point
    ay=k(j)*vel(j)^2;
    ayMax = 0.54728*(1/2)*cL*ro*A*(1/m)*muY*(vel(j)^2) + 0.038*cL*vel(j)+0.958*muY*g;
    axMaxGGV = 0;
    if abs(ay)<ayMax
        axMax = 1.05*muX*g + 0.5*(1/m)*ro*A*(muX*cL-cD)*1.51*(vel(j)^2);
        axMaxGGV = - axMax*sqrt(1-(ay/ayMax)^2);
    end
    axMaxController = - maxAccFactorController*(g + 0.5*ro*A*cL*(vel(i-1)^2))/m;
    axMaxMotors = - ax_br_scale_factor*((1/vel(i-1))*(P_br*1000*eta/m) - 0.5*(ro*A/m)*(cD*vel(i-1)^2));
    ax = - max(max(axMaxGGV, axMaxMotors),axMaxController);

    vel(j-1) = min(vel(j-1), sqrt(vel(j)^2 + 2*ax*ds(j-1)));
end

% Close loop
vel(m) = vel(1);

% Backwards loop. Second iteration
for j = N:-1:2
    % Compute ax given curvature and velocity at this point
    ay=k(j)*vel(j)^2;
    ayMax = 0.54728*(1/2)*cL*ro*A*(1/m)*muY*(vel(j)^2) + 0.038*cL*vel(j)+0.958*muY*g;
    axMaxGGV = 0;
    if abs(ay)<ayMax
        axMax = 1.05*muX*g + 0.5*(1/m)*ro*A*(muX*cL-cD)*1.51*(vel(j)^2);
        axMaxGGV = - axMax*sqrt(1-(ay/ayMax)^2);
    end
    axMaxController = - maxAccFactorController*(g + 0.5*ro*A*cL*(vel(i-1)^2))/m;
    axMaxMotors = - ax_br_scale_factor*((1/vel(i-1))*(P_br*1000*eta/m) - 0.5*(ro*A/m)*(cD*vel(i-1)^2));
    ax = - max(max(axMaxGGV, axMaxMotors),axMaxController);

    vel(j-1) = min(vel(j-1), sqrt(vel(j)^2 + 2*ax*ds(j-1)));
end

% Compute acceleration along trajectory: forward difference of kinetic energy -> a = (v^2_next - v^2)/ (2*ds)
acc = zeros(N,1);
for i = 1:N-1
    if ds(i+1) > 0
        acc(i) = (vel(i+1)^2 - vel(i)^2) / (2*ds(i+1));
    else
        acc(i) = 0;
    end
end
% For last point, use previous acceleration
acc(N) = acc(N-1);

trajectory = [x(:),y(:),psi(:),s(:),vel(:),acc(:),k(:)];

end