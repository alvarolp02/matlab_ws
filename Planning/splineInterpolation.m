function [smoothedPath] = splineInterpolation(path, resolution)


d = sqrt(sum(diff(path).^2, 2));
t = [0; cumsum(d,1)];

N = floor(t(end)/resolution);
tq = linspace(0, t(end), N);

xq = spline(t, path(:,1), tq);
yq = spline(t, path(:,2), tq);

smoothedPath = [xq(:), yq(:)];

end