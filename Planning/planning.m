[left,right] = extractBoundaries("trackBoundaries/FSG24.txt");
hold on
axis equal
set(gca,'XDir','reverse');

plot(left(:,2), left(:,1))
plot(right(:,2), right(:,1))

middle = middleLine(left,right);
plot(middle(:,2),middle(:,1),".g")
