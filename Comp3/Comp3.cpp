// Library for Forward and Inverse Kinematics
// By Adrian Fettes, Feb 2017

#include "Comp3.h"

double normalizeAngle(double angleRadians) {
	double temp = angleRadians;
	if (angleRadians > PI) {
		temp = angleRadians - 2*PI;
	}
	else if (angleRadians < -1 * PI) {
		temp = angleRadians + 2*PI;
	}
	return temp;
}

jointReturn INVKIN(frameParam_t &wrist) {
	double theta1a, theta2a, d3, theta4a, theta1b, theta2b, theta4b;
	jointReturn temp;

	int r = (wrist.x * wrist.x + wrist.y * wrist.y);
	if (r > (L3 + L4) * (L3 + L4) || r < (L3 - L4) * (L3 - L4)) {
		temp.configValid = 0;
		return temp;
	}

	d3 = L1 + L2 - wrist.z - (Lmax + L7 + L8/2);
	int v = ((wrist.x * wrist.x) + (wrist.y * wrist.y) - (L3 * L3) - (L4 * L4)) / (2 * L3 * L4);
	theta2a = acos(((wrist.x * wrist.x) + (wrist.y * wrist.y) - (L3 * L3) - (L4 * L4)) / (2 * L3 * L4));
	theta2b = -1 * theta2a;
	theta1a = atan2(wrist.y, wrist.x) - atan2(L4 * sin(theta2a), L3 + L4 * cos(theta2a));
	theta1b = atan2(wrist.y, wrist.x) - atan2(L4 * sin(theta2b), L3 + L4 * cos(theta2b));
	theta4a = theta1a + theta2a - DEG2RAD(wrist.theta);
	theta4b = theta1b + theta2b - DEG2RAD(wrist.theta);

	temp.config1[0] = RAD2DEG(normalizeAngle(theta1a));
	temp.config1[1] = RAD2DEG(normalizeAngle(theta2a));
	temp.config1[2] = d3;
	temp.config1[3] = RAD2DEG(normalizeAngle(theta4a));

	temp.config2[0] = RAD2DEG(normalizeAngle(theta1b));
	temp.config2[1] = RAD2DEG(normalizeAngle(theta2b));
	temp.config2[2] = d3;
	temp.config2[3] = RAD2DEG(normalizeAngle(theta4b));

	temp.configValid = 1;

	return temp;
}

jointReturn SOLVE(frameParam_t &tool) {
	frameParam_t temp = tool;
	temp.z = temp.z + L7 - L6 + L8/2;
	jointReturn tempJ = INVKIN(temp);
	
	return tempJ;
}