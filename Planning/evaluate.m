function [distance, meanCurvature] = evaluate(trajectory)
%EVALUATE Compute distance curvature and time given a [x,y] trajectory

diffs = diff(trajectory(:,1:2),1,1);      % differences between consecutive points
segLengths = hypot(diffs(:,1), diffs(:,2)); % euclidean lengths
distance = sum(segLengths);

% Curvature: (dx*ddy - dy*ddx)/(dx^2+dy^2)^(3/2)
dx=gradient(trajectory(:,1));
dy=gradient(trajectory(:,2));
ddx=gradient(dx);
ddy=gradient(dy);
curvature=(dx.*ddy - dy.*ddx)./((dx.^2+dy.^2).^(3/2));
meanCurvature = mean(abs(curvature));

end