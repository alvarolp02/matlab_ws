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
middle = middleLine(left,right,0.2);
[twL, twR] = trackWidths(left, right, middle, 0.5, "cone");
plot(middle(:,2),middle(:,1),".g")

% Compute normals
n = normals(middle);

% % Visualize track limits from normals and trackwidths
trackLimLeft = middle+n.*twL;
trackLimRight = middle-n.*twR;
plot(trackLimLeft(:,2),trackLimLeft(:,1),".b")
plot(trackLimRight(:,2),trackLimRight(:,1),".y")


% % MIN DIST BASED ON TRACK LIMITS
% deltaX = trackLimLeft(:,1)-trackLimRight(:,1);
% deltaY = trackLimLeft(:,2)-trackLimRight(:,2);
% alphaMinDistTL = minDistTrajTL(trackLimLeft, trackLimRight);
% plot(trackLimRight(:,2) + alphaMinDistTL.*deltaY,trackLimRight(:,1) + alphaMinDistTL.*deltaX, ".r")


% % MIN DIST
% ref = middle;
% n = normals(ref);
% [twL, twR] = trackWidths(left, right, ref, 0.5, "cone");
% alphaMinDist = minDistTraj(ref, n, twL, twR);
% plot(ref(:,2) + alphaMinDist.*n(:,2),ref(:,1) + alphaMinDist.*n(:,1), ".r")

% MIN CURVATURE
ref = middle;

nIt = 1;
distances=zeros(nIt,1);
meanCurvatures=zeros(nIt,1);
for i=1:nIt
    n = normals(ref);
    [twL, twR] = trackWidths(left, right, ref, 0.5, "cone");
    alphaMinCurv = minCurvTraj(ref, n, twL, twR);
    ref = ref + alphaMinCurv.*n;
    [distances(i),meanCurvatures(i)]=evaluate(ref);
end
racingLine=ref;

plot(racingLine(:,2), racingLine(:,1), ".r")

% Visualize tracklimits
trackLimLeft = ref + twL.*n;
trackLimRight = ref - twR.*n;
plot(trackLimLeft(:,2),trackLimLeft(:,1))
plot(trackLimRight(:,2),trackLimRight(:,1))

distances;
meanCurvatures;

trajectory = velProfile(racingLine)