// Library for Forward and Inverse Kinematics
// By Monica Li, Feb 2017


#include "Comp2.h"

int theta_1, theta_2, d_3, theta_4;
HomoMat JOINT_TO_MATRIX(int alpha, int a, int d, int theta);

// KIN: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Wrist Frame 
frameParam_t KIN(JOINT &conf) {
	// Assign configurations
	theta_1 = conf[0];
	theta_2 = conf[1];
	d_3 = conf[2];
	theta_4 = conf[3];

	// Define D-H frames
	int alpha[4] = { 0, 0, 0, 180 };
	int a[4] = { 0, 195, 142, 0 };
	int d[4] = { 405, 70, d_3, 270 };
	int theta[4] = { theta_1, theta_2, 0, theta_4 };

	// T matrices
	HomoMat T[4];
	for (int i = 0; i < 4; i++) {
		T[i] = JOINT_TO_MATRIX(alpha[i], a[i], d[i], theta[i]);
	}

	HomoMat T_Wrist = T[0] * T[1] * T[2];
	frameParam_t temp = T_Wrist.ITOU();
	return temp;
}

// WHERE: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Tool Frame 
frameParam_t WHERE(JOINT &conf) {
	// Assign configurations
	theta_1 = conf[0];
	theta_2 = conf[1];
	d_3 = conf[2];
	theta_4 = conf[3];

	// Define D-H frames
	int alpha[4] = { 0, 0, 0, 180 };
	int a[4] = { 0, 195, 142, 0 };
	int d[4] = { 405, 70, d_3, 270 };
	int theta[4] = { theta_1, theta_2, 0, theta_4 };

	// T matrices
	HomoMat T[4];
	for (int i = 0; i < 4; i++) {
		T[i] = JOINT_TO_MATRIX(alpha[i], a[i], d[i], theta[i]);
	}

	HomoMat T_Tool = T[0] * T[1] * T[2] * T[3];
	frameParam_t temp = T_Tool.ITOU();
	return temp;

}
// Convert joint parameters into T matrix
HomoMat JOINT_TO_MATRIX(int alpha, int a, int d, int theta) {
	HomoMat temp;
	temp.homoMatrix[0][0] = cos(DEG2RAD(theta));
	temp.homoMatrix[0][1] = -sin(DEG2RAD(theta));
	temp.homoMatrix[0][3] = -a;
	temp.homoMatrix[1][0] = sin(DEG2RAD(theta))*cos(DEG2RAD(alpha));
	temp.homoMatrix[1][1] = cos(DEG2RAD(theta))*cos(DEG2RAD(alpha));
	temp.homoMatrix[1][2] = -sin(DEG2RAD(alpha));
	temp.homoMatrix[1][3] = -sin(DEG2RAD(alpha))*d;
	temp.homoMatrix[2][0] = sin(DEG2RAD(theta))*sin(DEG2RAD(alpha));
	temp.homoMatrix[2][1] = cos(DEG2RAD(theta))*sin(DEG2RAD(alpha));
	temp.homoMatrix[2][2] = cos(DEG2RAD(alpha));
	temp.homoMatrix[2][3] = cos(DEG2RAD(alpha))*d;
	return temp;
}