function [n] = normals(path)
%NORMALS Compute normals along smooth path

dx = gradient(path(:,1));
dy = gradient(path(:,2));
dl = sqrt(dx.*dx+dy.*dy);
nx = -dy./dl;
ny = dx./dl;
n = [nx(:),ny(:)];

end