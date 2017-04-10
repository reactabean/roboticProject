function [ V] = Vfun( position, velocity )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
         l1 =0.405;
         l2 =0.07;
         l3 =0.195;
         l4 =0.142;
         l5 =0.14;
         l6 =0.08;
         l7 =0.13;
         l8 =0.01;
         l9 =0.03;
         lmax =0.41;

         frictCoeff =0.5;
         deltaT =0.2;

         M1 =1.7;
         M2 =1.0;
         M3 =1.7;
         M4 = 1.0;

         gravity = 9.8;


	theta1 = position(0);
	theta2 = position(1);
	D3 = position(2);
	theta4 = position(3);

	thetadot1 = velocity(0);
	thetadot2 = velocity(1);
	Ddot3 = velocity(2);
	thetadot4 = velocity(3);

	V(0) = -M4*l9*(sin(theta4)*(l4*(thetadot1*thetadot1) + l4*(thetadot2*thetadot2) + l3*(thetadot1*thetadot1)*cos(theta2) + l4*thetadot1*thetadot2*2.0) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
	V(1) = -M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
	V(2) = 0;
	V(3) = M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2));


end

