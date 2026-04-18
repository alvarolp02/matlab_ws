function [middle, twL, twR] = middleLine(left,right,minDist, interpDist, boundaryType)
%MIDDLELINE Compute smooth middleLine given left and right boundaries.
% Generate also the trackWidths considering minDist to boundaries
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

if boundaryType=="border"
    factor = 20;   % puntos intermedios por segmento
    
    left_fine = [];
    right_fine = [];
    t = linspace(0,1,factor);
    for i = 1:size(left,1)-1
        segment = (1-t)' * left(i,:) + t' * left(i+1,:);
        left_fine = [left_fine; segment];
    end
    segment = (1-t)' * left(end,:) + t' * left(1,:);
    left_fine = [left_fine; segment];
    for i = 1:size(right,1)-1
        segment = (1-t)' * right(i,:) + t' * right(i+1,:);
        right_fine = [right_fine; segment];
    end
    segment = (1-t)' * right(end,:) + t' * right(1,:);
    right_fine = [right_fine; segment];
    left = left_fine;
    right = right_fine;
end
    
idxL = knnsearch(left(:,1:2), middle(:,1:2));
idxR = knnsearch(right(:,1:2), middle(:,1:2));
twL = zeros(size(middle, 1), 1);
twR = zeros(size(middle, 1), 1);

for i=1:size(middle,1)
    twL(i) = max(norm(middle(i,:)-left(idxL(i),:))-minDist, 0.05);
    twR(i) = max(norm(middle(i,:)-right(idxR(i),:))-minDist, 0.05);
end

end