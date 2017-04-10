% outputFile = csvread('../pureTorqueOutput.csv', 1, 0);
outputFile = csvread('../simulatorOutput.csv', 1, 0);
outputSpline = csvread('../currentPosition.csv', 1, 0);

time = outputFile(:, 1);
positions = outputFile(:, 2:5);
velocities = outputFile(:, 6:9);
accels = outputFile(:, 10:13);
tau = outputFile(:, 14:17);

theta1spline = outputSpline(6:4:18, 1:4);
theta2spline = outputSpline(7:4:19, 1:4);
distance3spline = outputSpline(8:4:20, 1:4);
theta4spline = outputSpline(9:4:21, 1:4);

[positionSplineTheta1, velSplineTheta1, accSplineTheta1] = calcSplineTrajVelAcc(theta1spline, size(time, 1));
[positionSplineTheta2, velSplineTheta2, accSplineTheta2] = calcSplineTrajVelAcc(theta2spline, size(time, 1));
[positionSplineDistance3, velSplineDistance3, accSplineDistance3] = calcSplineTrajVelAcc(distance3spline, size(time, 1));
[positionSplineTheta4, velSplineTheta4, accSplineTheta4] = calcSplineTrajVelAcc(theta4spline, size(time, 1));


posLimits = [-150, -100, -200, -160; 150, 100, -100, 160];
myPosSpline = [positionSplineTheta1, positionSplineTheta2, positionSplineDistance3, positionSplineTheta4];
quadplot(positions, myPosSpline, time, posLimits, 1.2, 'Position');

velLimits = [-150, -150, -50, -150; 150, 150, 50, 150];
myVelSpline = [velSplineTheta1, velSplineTheta2, velSplineDistance3, velSplineTheta4];
quadplot(velocities, myVelSpline, time, velLimits, 1.2, 'Velocity');

accelLimits = [-600, -600, -200, -600; 600, 600, 200, 600];
myAccSpline = [accSplineTheta1, accSplineTheta2, accSplineDistance3, accSplineTheta4];
quadplot(accels, myAccSpline, time, accelLimits, 1.2, 'Acceleration');

torqueLimits = [-16, -16, -45, -16; 16, 16, 45, 16];
quadplot(tau, tau, time, torqueLimits, 1.2, 'Torque');