
syms theta1 theta2 theta4 d3;
syms L1 L2 L3 L4 L5 L6 L7 L8 Lmax;
c1 = cos(theta1);
s1 = sin(theta1);
c2 = cos(theta2);
s2 = sin(theta2);
c4 = cos(theta4);
s4 = sin(theta4);

T0 = [c1 -s1 0 0; s1 c1 0 0; 0 0 1 L1; 0 0 0 1];
T1 = [c2 -s2 0 L3; s2 c2 0 0; 0 0 1 L2; 0 0 0 1];
T2 = [1 0 0 L4; 0 1 0 0; 0 0 1 -(Lmax+d3-L5); 0 0 0 1];
T3 = [c4 -s4 0 0;-s4 -c4 0 0; 0 0 -1 -(L5+L6); 0 0 0 1];
T4 = [1 0 0 0; 0 1 0 0; 0 0 1 L7-L6+L8/2 ; 0 0 0 1];

TS = T0 * T1 * T2 * T3 * T4

