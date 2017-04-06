syms gravity M1 M2 M3 M4 L1 L2 L3 L4 L5 L6 L9 Lmax theta1 theta2 D3 theta4 thetadot1 thetadot2 thetadotdot1 thetadotdot2 Ddot3 Ddotdot3 thetadot4 thetadotdot4;

R1 = [cos(theta1), -sin(theta1), 0; sin(theta1), cos(theta1), 0; 0, 0, 1];
R2 = [cos(theta2), -sin(theta2), 0; sin(theta2), cos(theta2), 0; 0, 0, 1];
R3 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
R4 = [cos(theta4), -sin(theta4), 0; -sin(theta4), -cos(theta4), 0; 0, 0, -1];
rotationMatrices = sym(zeros(3, 3, 4));
rotationMatrices(:, :, 1) = R1;
rotationMatrices(:, :, 2) = R2;
rotationMatrices(:, :, 3) = R3;
rotationMatrices(:, :, 4) = R4;

jointVelocities = [thetadot1, thetadot2, 0, thetadot4];
jointAccelerations = [thetadotdot1, thetadotdot2, 0, thetadotdot4];
prismaticVelocity = [0, 0, Ddot3, 0];
prismaticAcceleration = [0, 0, Ddotdot3, 0];
jointMasses = [M1, M2, M3, M4];
centersOfMass = [0 0 0 L9; 0 0 0 0; 0 0 0 0];
nextOrigin = [0, L3, L4, 0; 0, 0, 0, 0; L1, L2, -(Lmax+D3-L5), -L5-L6]; 
jointMoments = sym(zeros(3, 3, 4));
jointMoments(:, :, 4) = [0, 0, 0; 0, M4*L9^2, 0; 0, 0, M4*L9^2];

[myf, myn] = calculateTorques( rotationMatrices, jointVelocities, jointAccelerations, jointMasses, nextOrigin, centersOfMass, jointMoments, prismaticVelocity, prismaticAcceleration );

% RUN THESE LINES WITHOUT THE SEMICOLON IN THE CONSOLE TO GET THE FINAL RESULT
ccode(simplify(myn(3, 1))); % This is the torque at joint 1
ccode(simplify(myn(3, 2))); % This is the torque at joint 2
ccode(simplify(myf(3, 3))); % This is the force at joint 3
ccode(simplify(myn(3, 4))); % This is the torque at joint 4

poly1 = simplify(myn(3, 1));
poly2 = simplify(myn(3, 2));
poly3 = simplify(myf(3, 3));
poly4 = simplify(myn(3, 4));

 M00 = collect(poly1, thetadotdot1);
 M01 = collect(poly1, thetadotdot2);
 M02 = collect(poly1, Ddotdot3);
 M03 = collect(poly1, thetadotdot4);
 
 M10 = collect(poly2, thetadotdot1);
 M11 = collect(poly2, thetadotdot2);
 M12 = collect(poly2, Ddotdot3);
 M13 = collect(poly2, thetadotdot4);
 
 M20 = collect(poly3, thetadotdot1);
 M21 = collect(poly3, thetadotdot2);
 M22 = collect(poly3, Ddotdot3);
 M23 = collect(poly3, thetadotdot4);
 
 M30 = collect(poly4, thetadotdot1);
 M31 = collect(poly4, thetadotdot2);
 M32 = collect(poly4, Ddotdot3);
 M33 = collect(poly4, thetadotdot4);
 
 poly1_nodotdot =  subs(poly1, [thetadotdot1,thetadotdot2, Ddotdot3, thetadotdot4], [0,0,0,0]);
 poly2_nodotdot =  subs(poly2, [thetadotdot1,thetadotdot2, Ddotdot3, thetadotdot4], [0,0,0,0]);
 poly3_nodotdot =  subs(poly3, [thetadotdot1,thetadotdot2, Ddotdot3, thetadotdot4], [0,0,0,0]);
 poly4_nodotdot =  subs(poly4, [thetadotdot1,thetadotdot2, Ddotdot3, thetadotdot4], [0,0,0,0]);

 g0 = subs(poly1_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
 g1 = subs(poly2_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
 g2 = subs(poly3_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
 g3 = subs(poly4_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
 
 V0 = poly1_nodotdot - g0;
 V1 = poly2_nodotdot - g1;
 V2 = poly3_nodotdot - g2;
 V3 = poly4_nodotdot - g3;
 
v0 = ccode(simplify(V0));
v1 = ccode(simplify(V1));
v2 = ccode(simplify(V2));
v3 = ccode(simplify(V3));
 
 
 