syms M1 M2 M3 L1 L2 L3 L4 L5 L6 Lmax theta1 theta2 thetadot1 thetadot2 thetadotdot1 thetadotdot2 Ddot3 Ddotdot3;

R1 = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
R2 = [cos(theta2), -sin(theta2), 0; sin(theta2), cos(theta2), 0; 0, 0, 1];
R3 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
rotationMatrices = sym(zeros(3, 3, 3));
rotationMatrices(:, :, 1) = R1;
rotationMatrices(:, :, 2) = R2;
rotationMatrices(:, :, 3) = R3;

jointVelocities = [thetadot1, thetadot2, 0];
jointAccelerations = [thetadotdot1, thetadotdot2, 0];
prismaticVelocity = [0, 0, Ddot3];
prismaticAcceleration = [0, 0, Ddotdot3];
jointMasses = [M1, M2, M3];
centersOfMass = [0 0 0; 0 0 0; 0 0 0];
nextOrigin = [0, L3, L4; 0, 0, 0; 0, L2, -(Lmax-L5)]; 
jointMoments = zeros(3, 3);

[myf, myn] = calculateTorques( rotationMatrices, jointVelocities, jointAccelerations, jointMasses, nextOrigin, centersOfMass, jointMoments, prismaticVelocity, prismaticAcceleration );

% RUN THESE LINES WITHOUT THE SEMICOLON IN THE CONSOLE TO GET THE FINAL RESULT
ccode(simplify(myn(3, 1))); % This is the torque at joint 1
ccode(simplify(myn(3, 2))); % This is the torque at joint 2
ccode(simplify(myf(3, 3))); % This is the force at joint 3