function [totalHits, conesDetected, hitsByCone] = calculateConesHit(conePositions, beamDistr, heightArray, angleArray, detect)

if nargin < 5 
    detect = 2;
end

totalHits = zeros(length(heightArray),length(angleArray));
conesDetected = zeros(length(heightArray),length(angleArray));
hitsByCone = zeros(length(heightArray),length(angleArray), length(conePositions));
for i=1:length(heightArray)
    h = heightArray(i);
    for j=1:length(angleArray)
        a=angleArray(j);
        hit = zeros(size(conePositions));
        for n = 1:length(beamDistr)
    
            y = h + tan((pi/180)*(beamDistr(n)+a))*conePositions;
    
            yhit = (y > 0) .* (y <= 0.325) .* (conePositions > 0);
            hit = hit + yhit;
        end
        totalHits(i,j)=sum(hit);
        conesDetected(i,j)=sum(hit>=detect);
        hitsByCone(i,j,:)=hit;
    end
end
end