function [left,right] = extractBoundaries(filePath)
%EXTRACT_BOUNDARIES extract left and right boundaries (cones) from
%middleline msg saved on txt file

fid = fopen(filePath, 'r');
if fid == -1
    error('Could not open file: %s', filePath);
end
% Read all lines into a cell array
lines = {};
tline = fgetl(fid);
while ischar(tline)
    lines{end+1,1} = tline; %#ok<AGROW>
    tline = fgetl(fid);
end
fclose(fid);

readingLeftX = false;
readingLeftY = false;
readingRightX = false;
readingRightY = false;
leftX = [];
leftY = [];
rightX = [];
rightY = [];

for i = 1:numel(lines)
    l = lines{i};
    if contains(l, 'left_boundary_x')
        readingLeftX = true;
        continue;
    elseif contains(l, 'left_boundary_y')
        readingLeftY = true;
        continue;
    elseif contains(l, 'right_boundary_x')
        readingRightX = true;
        continue;
    elseif contains(l, 'right_boundary_y')
        readingRightY = true;
        continue;
    end

    l = l(3:end);
    l = str2double(l); % Convert string to double for numerical storage
    if isnan(l)
        continue
    end

    if readingRightY
        rightY(end+1)=l;
    elseif readingRightX
        rightX(end+1)=l;
    elseif readingLeftY
        leftY(end+1)=l;
    elseif readingLeftX
        leftX(end+1)=l;
    end
   
end

% Combine into outputs (ensure column vectors)
left = [leftX(:), leftY(:)];
right = [rightX(:), rightY(:)];

end