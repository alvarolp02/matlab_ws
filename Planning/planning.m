[left,right] = extractBoundaries("trackBoundaries/FSG24.txt");
hold on
axis equal
set(gca,'XDir','reverse');

plot(left(:,2), left(:,1))
plot(right(:,2), right(:,1))