function [middle, twL, twR] = middleLine(left,right,minDist)
%MIDDLELINE Compute smooth middleLine given left and right boundaries.
%Generate also the trackWidths considering minDist to boundaries

% For efficiency, use KD-tree via knnsearch (Statistics and ML toolbox)
idx = knnsearch(right(:,1:2), left(:,1:2));
middle = (left + right(idx,:)) / 2;


middle(end+1,:)=middle(1,:); % Close middle points

interpDist = 0.1; % 10 cm

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

idxL = knnsearch(left(:,1:2), middle(:,1:2));
idxR = knnsearch(right(:,1:2), middle(:,1:2));
twL = zeros(size(middle, 1), 1);
twR = zeros(size(middle, 1), 1);

for i=1:size(middle,1)
    twL(i) = max(norm(middle(i,:)-left(idxL(i),:))-minDist, 0.05);
    twR(i) = max(norm(middle(i,:)-right(idxR(i),:))-minDist, 0.05);
end

end