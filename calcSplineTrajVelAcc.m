function [ outputPos, outputVel, outputAcc ] = calcSplineTrajVelAcc( splineCoeffs, numSamples)
syms x;
num = numSamples;
while (mod(num, 4) > 0)
    num = num+1;
end

samplePoints = linspace(0, 4, num);

for i = 1:4
    mySpline = splineCoeffs(i, 1) + splineCoeffs(i, 2)*(x-i+1) + splineCoeffs(i, 3)*(x-i+1)^2 + splineCoeffs(i, 4)*(x-i+1)^3;
    lowrange = floor((size(samplePoints, 2)*(i-1))/4) + 1;
    highrange = floor((size(samplePoints, 2)*(i))/4);
    outputPos(1, lowrange:highrange) = eval(subs(mySpline, x, samplePoints(lowrange:highrange)));
    outputVel(1, lowrange:highrange) = eval(subs(diff(mySpline), x, samplePoints(lowrange:highrange)));
    outputAcc(1, lowrange:highrange) = eval(subs(diff(mySpline, 2), x, samplePoints(lowrange:highrange)));
end

outputPos = outputPos.';
outputVel = outputVel.';
outputAcc = outputAcc.';


end

