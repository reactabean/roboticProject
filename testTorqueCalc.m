syms M1 M2 L1 L2 theta1 theta2 thetadot1 thetadot2 thetadotdot1 thetadotdot2;

R1 = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
R2 = [cos(theta2), -sin(theta2), 0; sin(theta2), cos(theta2), 0; 0, 0, 1];
rotationMatrices = sym(zeros(3, 3, 2));
rotationMatrices(:, :, 1) = R1;
rotationMatrices(:, :, 2) = R2;

jointVelocities = [thetadot1, thetadot2];
jointAccelerations = [thetadotdot1, thetadotdot2];
jointMasses = [M1, M2];
centersOfMass = [L1, L2; 0 0; 0 0];
nextOrigin = [0, L1; 0 0; 0 0];
jointMoments = zeros(3,3, 2);

[myf, myn] = calculateTorques( rotationMatrices, jointVelocities, jointAccelerations, jointMasses, nextOrigin, centersOfMass, jointMoments, [0 0], [0 0] );

