function [middle] = middleLine(left, right, interpDist)
%MIDDLELINE Compute smooth middleLine given left and right boundaries.
% left: left boundary cone positions
% right: right boundary cone positions
% minDist: minimum distance to cones
% interpDist: resolution of resulting middleline
% boundaryType: "cone" or "border", decide which to avoid

% For efficiency, use KD-tree via knnsearch (Statistics and ML toolbox)
idx = knnsearch(right(:,1:2), left(:,1:2));
middle = (left + right(idx,:)) / 2;


middle(end+1,:)=middle(1,:); % Close middle points

% compute cumulative arc-length parameter t for 'middle' (already closed)
deltas = sqrt(sum(diff(middle).^2,2));
totalDist = sum(deltas);

% create normalized parameter t along middle points
t = [0; cumsum(deltas)] / totalDist;

% number of samples for interpDist spacing (ensure at least 2 points)
nSamples = max(2, ceil(totalDist / interpDist) + 1);

% interpolate coordinates vs arc-length parameter using shape-preserving pchip
xs = interp1(t, middle(:,1), linspace(0,1,nSamples), 'pchip');
ys = interp1(t, middle(:,2), linspace(0,1,nSamples), 'pchip');

middle = [xs(:),ys(:)];

end