function [distance, meanCurvature] = evaluatePath(path)
%EVALUATE Compute distance and curvature given a [x,y] path

diffs = diff(path(:,1:2),1,1);      % differences between consecutive points
segLengths = hypot(diffs(:,1), diffs(:,2)); % euclidean lengths
distance = sum(segLengths);

% Curvature: (dx*ddy - dy*ddx)/(dx^2+dy^2)^(3/2)
dx=gradient(path(:,1));
dy=gradient(path(:,2));
ddx=gradient(dx);
ddy=gradient(dy);
curvature=(dx.*ddy - dy.*ddx)./((dx.^2+dy.^2).^(3/2));
meanCurvature = mean(abs(curvature));

end