clearvars
close

% Load boundary cones from message
[left,right] = extractBoundaries("trackBoundaries/FSG24.txt");
hold on
axis equal
set(gca,'XDir','reverse');

plot(left(:,2), left(:,1), ".b", MarkerSize=20)
plot(right(:,2), right(:,1), ".y", MarkerSize=20)

% Compute middleline and track widths
[middle, twL, twR] = middleLine(left,right,0.5);
plot(middle(:,2),middle(:,1),".g")

% Compute normals
n = normals(middle);

% Visualize track limits from normals and trackwidths
trackLimLeft = middle+n.*twL;
trackLimRight = middle-n.*twR;
plot(trackLimLeft(:,2),trackLimLeft(:,1),".b")
plot(trackLimRight(:,2),trackLimRight(:,1),".y")






