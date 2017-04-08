syms gravity M1 M2 M3 M4 l1 l2 l3 l4 l5 l6 l9 lmax theta1 theta2 D3 theta4 thetadot1 thetadot2 thetadotdot1 thetadotdot2 Ddot3 Ddotdot3 thetadot4 thetadotdot4;

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
centersOfMass = [0 0 0 l9; 0 0 0 0; 0 0 0 0];
nextOrigin = [0, l3, l4, 0; 0, 0, 0, 0; l1, l2, -(lmax+D3-l5), -l5-l6]; 
jointMoments = sym(zeros(3, 3, 4));
jointMoments(:, :, 4) = [0, 0, 0; 0, M4*l9^2, 0; 0, 0, M4*l9^2];

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

G0 = subs(poly1_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
G1 = subs(poly2_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
G2 = subs(poly3_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);
G3 = subs(poly4_nodotdot, [thetadot1, thetadot2, Ddot3,thetadot4], [0,0,0,0]);

V0 = poly1_nodotdot - G0;
V1 = poly2_nodotdot - G1;
V2 = poly3_nodotdot - G2;
V3 = poly4_nodotdot - G3;

%% remove the semicolon to show the result for c code
m00 = ccode(simplify(M00));
m01 = ccode(simplify(M01));
m02 = ccode(simplify(M02));
m03 = ccode(simplify(M03));

m10 = ccode(simplify(M10));
m11 = ccode(simplify(M11));
m12 = ccode(simplify(M12));
m13 = ccode(simplify(M13));

m20 = ccode(simplify(M20));
m21 = ccode(simplify(M21));
m22 = ccode(simplify(M22));
m23 = ccode(simplify(M23));

m30 = ccode(simplify(M30));
m31 = ccode(simplify(M31));
m32 = ccode(simplify(M32));
m33 = ccode(simplify(M33));

g0 = ccode(simplify(G0));
g1 = ccode(simplify(G1));
g2 = ccode(simplify(G2));
g3 = ccode(simplify(G3));

v0 = ccode(simplify(V0));
v1 = ccode(simplify(V1));
v2 = ccode(simplify(V2));
v3 = ccode(simplify(V3));

%% Inverse matrix 
im_00 = M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2)) + M2*(l3*l3)*cos(theta2*2.0);
im_01 = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
im_02 = 0;
im_03 = -2 * l9 * l9 * M4;

im_10 = M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
im_11 = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
im_12 = 0;
im_13 = -2 * l9 * l9 * M4;

im_20 = 0;
im_21 = 0;
im_22 = M3 + M4;
im_23 = 0;

im_30 = -(M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)));
im_31 = -(M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4)));
im_32 = 0;
im_33 = 2 * l9 * l9 * M4;

Mass_matrix = [im_00 im_01 im_02 im_03;
               im_10 im_11 im_12 im_13;
               im_20 im_21 im_22 im_23;
               im_30 im_31 im_32 im_33];
           
inverse_mass_matrix = inv(Mass_matrix);
ccode(simplify(inverse_mass_matrix));
        
  
 
 
 