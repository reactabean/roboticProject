function [ M ] = Mfun( position,velocity )
%UNTITLED4 Summary of this function goes here
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
		theta4 = position*(3);

		thetadot1 = velocity(0);
		thetadot2 = velocity(1);
		Ddot3 = velocity(2);
		thetadot4 = velocity(3);	

		M(0,0) = M2*(l3*l3) + M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
		M(0,1) = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
		M(0,2) = 0;
		M(0,3) = -2 * l9 * l9 * M4;

		M(1,0) = M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
		M(1,1) = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
		M(1,2) = 0;
		M(1,3) = -2 * l9 * l9 * M4;

		M(2,0) = 0;
		M(2,1) = 0;
		M(2,2) = M3 + M4;
		M(2,3) = 0;

		M(3,0) = -(M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)));
		M(3,1) = -(M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4)));
		M(3,2) = 0;
		M(3,3) = 2 * l9 * l9 * M4;
end

