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

vel = zeros(N,1);
acc = zeros(N,1);

trajectory = [x(:),y(:),psi(:),s(:),vel(:),acc(:),k(:)];

end