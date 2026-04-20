function [twL, twR] = trackWidths(left, right, reference, minDist, boundaryType)
%TRACKWIDTHS Generate the trackWidths considering minDist to boundaries
% left: left boundary cone positions
% right: right boundary cone positions
% reference: trajectory used as reference for tw generation
% minDist: minimum distance to cones
% boundaryType: "cone" or "border", decide which to avoid

if boundaryType=="border"
    factor = 20; % In-between points per segment
    
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
    
idxL = knnsearch(left(:,1:2), reference(:,1:2));
idxR = knnsearch(right(:,1:2), reference(:,1:2));
twL = zeros(size(reference, 1), 1);
twR = zeros(size(reference, 1), 1);

for i=1:size(reference,1)
    twL(i) = max(norm(reference(i,:)-left(idxL(i),:))-minDist, 0.05);
    twR(i) = max(norm(reference(i,:)-right(idxR(i),:))-minDist, 0.05);
end

end