// Library for Forward and Inverse Kinematics
// By Adrian Fettes, Feb 2017

#include "Comp3.h"

jointReturn INVKIN(frameParam_t &wrist) {
	double theta1a, theta2a, d3, theta4a, theta1b, theta2b, theta4b;
	jointReturn temp;

	d3 = L1 + L2 - wrist.z - (Lmax + L7 + L8);
	int v = ((wrist.x * wrist.x) + (wrist.y * wrist.y) - (L3 * L3) - (L4 * L4)) / (2 * L3 * L4);
	theta2a = acos(((wrist.x * wrist.x) + (wrist.y * wrist.y) - (L3 * L3) - (L4 * L4)) / (2 * L3 * L4));
	theta2b = -1 * theta2a;
	theta1a = atan2(wrist.y, wrist.x) - atan2(L4 * sin(theta2a), L3 + L4 * cos(theta2a));
	theta1b = atan2(wrist.y, wrist.x) - atan2(L4 * sin(theta2b), L3 + L4 * cos(theta2b));
	theta4a = wrist.theta - theta1a - theta2a;
	theta4b = wrist.theta - theta1b - theta2b;

	temp.config1[0] = theta1a;
	temp.config1[1] = theta2a;
	temp.config1[2] = d3;
	temp.config1[3] = theta4a;

	temp.config2[0] = theta1b;
	temp.config2[1] = theta2b;
	temp.config2[2] = d3;
	temp.config2[3] = theta4b;
	return temp;
}

jointReturn SOLVE(frameParam_t &tool) {
	jointReturn temp = INVKIN(tool);
	
	return temp;
}