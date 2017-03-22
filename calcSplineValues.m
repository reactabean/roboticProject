function [ splineValues ] = calcSplineValues( abcd, resolution, size )
points = zeros(size, resolution/4);

for i = 0:size-1
    points(i+1, :) = linspace(i, i+1, resolution/4);
    splineValues((i*resolution/4)+1:((i+1)*resolution/4)) = abcd(i+1, 1) + abcd(i+1, 2)*(points(i+1, :)-i) + abcd(i+1, 3)*(points(i+1, :)-i).^2 + abcd(i+1, 4)*(points(i+1, :)-i).^3;
end

end

