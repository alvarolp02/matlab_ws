close
clearvars

% beamDistr= [15 11 8 5 3 2 1.6666 1.3333 1 0.6666 0.3333 0 -0.3333 -0.6666 -1 -1.3333 -1.6666 -2 -2.3333 -2.6666 -3 -3.3333 -3.6666 -4 -4.3333 -4.6666 -5 -5.3333 -5.6666 -6 -7 -8 -9 -10 -11 -12 -13 -14 -19 -25];
% beamDistr = linspace(-21.95,21.95,256); %OS1 Max 256
% beamDistr = linspace(-21.95,21.95,128); %OS1 Max 128
% beamDistr = linspace(-21.95,0.0,128); %OS1 Max 128 below horizon
beamDistr = [-25,-18.9,linspace(-14,-7,8),linspace(-6,2,49),3,5,8,11,15]; %Pandar64
% beamDistr = [-25,linspace(-24.1,-6.6,37),linspace(-6.1,2.013,65),linspace(2.512,13.5,24),14.43]; %Pandar128
% beamDistr = [-24.7,-23,-21.65,20.37,-19.7,-19,-18.4,linspace(-17.7,-6.37,32),linspace(-6,2.095,66),linspace(2.463,13.3,22),14.98]; %OT128
beamDistr = round(beamDistr,2);

degresstring = compose('%.2f',beamDistr);
degresstring = strcat(degresstring,'°');

promptHeight = 'At what height should the lidar be placed [in m]? ';
lidarHeight = input(promptHeight);


promptbeamDistr = 'How many degrees should the lidar be rotated [in °]? ';
mountingAngle = input(promptbeamDistr);

conePositions = 0:1:150; %Cone positions
[totalHits, conesDetected, hitsByCone] = calculateConesHit(conePositions, beamDistr, lidarHeight, mountingAngle, 2);
for n = 1:length(beamDistr)
    y = lidarHeight + tan((pi/180)*(beamDistr(n)+mountingAngle))*conePositions;
    plot(conePositions,y)
    hold on;
end
xlabel('Distance in m')
ylabel('Lidar height in m')
ylim([0 lidarHeight+0.2])
axis("equal")

% Display cones with a height of 0.325m
for a = conePositions(2:end)
    line([a a], [0 0.325],'LineWidth',4);
end

hitsByCone(1) = [];
legend(degresstring);
figure();
for m = 1:length(hitsByCone)
    line([m m], [0 hitsByCone(m)]);
    hold on;
end
grid on;
line([1 1], [0 hitsByCone(1)]);
hitString = num2str(totalHits);
hittext = strcat('Cone hits: ',hitString);
detectedString = num2str(conesDetected);
detectedtext = strcat('Detected cones (2+ hits): ',detectedString);
xlabel('Cones distances')
ylabel('Times a cone has been hit')
annotation('textbox',[.65 .7 .1 .1],'String',detectedtext,'Edgecolor','r');
annotation('textbox',[.65 .8 .1 .1],'String',hittext,'Edgecolor','r');
