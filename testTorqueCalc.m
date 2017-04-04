syms M1 M2 L1 L2 theta1 theta2 thetadot1 thetadot2 thetadotdot1 thetadotdot2;

R1 = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
R2 = [cos(theta2), -sin(theta2), 0; sin(theta2), cos(theta2), 0; 0, 0, 1];
rotationMatrices(:, :, 1) = R1;
rotationMatrices(:, :, 1) = R2;

jointVelocities = [thetadot1, thetadot2; 0 0; 0 0];
jointAccelerations = [thetadotdot1, thetadotdot2; 0 0; 0 0];
jointMasses = [M1, M2; 0 0; 0 0];
centersOfMass1 = [L1, L2; 0 0; 0 0]; % Fix me 
centersOfMass2 = [L1, L2; 0 0; 0 0]; % Fix me
jointMoments = zeros(3, 2);

[myf, myn] = calculateTorques( rotationMatrices, jointVelocities, jointAccelerations, jointMasses, centersOfMass1, centersOfMass2, jointMoments );

