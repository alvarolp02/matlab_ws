clearvars
close

global conePositions;
conePositions = 0:1:200;

global beamDistr;
% beamDistr= [15 11 8 5 3 2 1.6666 1.3333 1 0.6666 0.3333 0 -0.3333 -0.6666 -1 -1.3333 -1.6666 -2 -2.3333 -2.6666 -3 -3.3333 -3.6666 -4 -4.3333 -4.6666 -5 -5.3333 -5.6666 -6 -7 -8 -9 -10 -11 -12 -13 -14 -19 -25];

beamDistr = linspace(-21.95,21.95,256); %OS1 Max 256
% beamDistr = linspace(-21.95,21.95,128); %OS1 Max 128
% beamDistr = linspace(-21.95,0.0,128); %OS1 Max 128 below horizon
% beamDistr = [-25,-18.9,linspace(-14,-7,8),linspace(-6,2,49),3,5,8,11,15]; %Pandar64
% beamDistr = [-25,linspace(-24.1,-6.6,37),linspace(-6.1,2.013,65),linspace(2.512,13.5,24),14.43]; %Pandar128
% beamDistr = [-24.7,-23,-21.65,20.37,-19.7,-19,-18.4,linspace(-17.7,-6.37,32),linspace(-6,2.095,66),linspace(2.463,13.3,22),14.98]; %OT128
beamDistr = round(beamDistr,2);

Nh = 50;
Na = 50;

%promptHeightStart = 'At what height should the lidar simulation start [in m]? ';
%yStart = input(promptHeightStart);
yStart = 0.1;
%promptHeightEnd = 'At what height should the lidar simulation end [in m]? ';
%yEnd = input(promptHeightEnd);
yEnd = 1.2;
heightInterval = (yEnd-yStart)/Nh;
heightArray = yStart:heightInterval:yEnd;

%promptbeamDistrStart = 'What is the start lidar alignment [in °]? ';
%angleStart = input(promptbeamDistrStart);
angleStart = -20;
%promptbeamDistrEnd = 'What is the end lidar alignment [in °]? ';
%angleEnd = input(promptbeamDistrEnd);
angleEnd = 20;
angleInterval = (angleEnd-angleStart)/Na;
angleArray = angleStart:angleInterval:angleEnd;

[z, conesDetected, hitsByCone] = calculateConesHit(conePositions, beamDistr, heightArray, angleArray, 2);
figure();
bar3(conesDetected);

xIdx = round(linspace(1, length(angleArray), 11));
yIdx = round(linspace(1, length(heightArray), 11));

xticks(xIdx);
yticks(yIdx);

xticklabels(angleArray(xIdx));
yticklabels(heightArray(yIdx));

xlabel('Angle');
ylabel('Height');
zlabel('Cones detected');
zlim([0 150]);
title("OS1 Max 256")

maxVal = max(conesDetected(:));

[row, col] = find(conesDetected == maxVal);

fprintf('Best positions:\n');
for k = 1:length(row)
    fprintf('Height: %.2f, Angle: %.2f, Cones detected: %g\n', ...
        heightArray(row(k)), angleArray(col(k)), conesDetected(row(k), col(k)));
end


