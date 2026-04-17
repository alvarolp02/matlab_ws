function [middle] = middleLine(left,right)
%MIDDLELINE Compute smooth middleLine given left and right boundaries

% Preallocate middle points
n = size(left,1);
middle = zeros(n,2);

% For efficiency, use KD-tree via knnsearch if available, otherwise pdist2
if exist('knnsearch','file')
    idx = knnsearch(right(:,1:2), left(:,1:2));
    middle = (left + right(idx,:)) / 2;
else
    % compute pairwise distances in chunks to avoid huge memory use
    for i = 1:n
        d = sum((right - left(i,:)).^2, 2);
        [~, j] = min(d);
        middle(i,:) = (left(i,:) + right(j,:)) / 2;
    end
end

middle(end+1,:)=middle(1,:); % Close middle points

interpDist = 0.1; % 10 cm

% compute cumulative arc-length parameter t for 'middle' (already closed)
deltas = sqrt(sum(diff(middle).^2,2));
totalDist = sum(deltas);

% create normalized parameter t along middle points
t = [0; cumsum(deltas)] / totalDist;

% number of samples for interpDist spacing (ensure at least 2 points)
nSamples = max(2, ceil(totalDist / interpDist) + 1);
tfine = linspace(0,1,nSamples);

% interpolate coordinates vs arc-length parameter using shape-preserving pchip
xs = interp1(t, middle(:,1), tfine, 'pchip');
ys = interp1(t, middle(:,2), tfine, 'pchip');

middle = [xs(:),ys(:)];

end