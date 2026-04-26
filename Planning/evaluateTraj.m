function [distance, meanCurvature, time] = evaluateTraj(trajectory)
%EVALUATE Compute distance and curvature given a [x,y] path

s = trajectory(:,4);
vel = trajectory(:,5);
k = trajectory(:,7);
distance = s(end);
meanCurvature = mean(abs(k));
ds = gradient(s);
time = sum(ds./vel);

end