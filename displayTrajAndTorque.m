outputFile = csvread('../pureTorqueOutput.csv', 1, 0);

time = outputFile(:, 1);
positions = outputFile(:, 2:5);
velocities = outputFile(:, 6:9);
accels = outputFile(:, 10:13);
tau = outputFile(:, 14:17);

posLimits = [-150, -100, -200, -160; 150, 100, -100, 160];
quadplot(positions, positions, time, posLimits, 1.2, 'Position');

velLimits = [-150, -150, -50, -150; 150, 150, 50, 150];
quadplot(velocities, velocities, time, velLimits, 1.2, 'Velocity');

accelLimits = [-600, -600, -200, -600; 600, 600, 200, 600];
quadplot(accels, accels, time, accelLimits, 1.2, 'Acceleration');

torqueLimits = [-16, -16, -45, -16; 16, 16, 45, 16];
quadplot(tau, tau, time, torqueLimits, 1.2, 'Torque');