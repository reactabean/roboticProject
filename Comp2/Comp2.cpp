// Library for Forward and Inverse Kinematics
// By Monica Li, Feb 2017


#include "Comp2.h"

// Transformation matrices
HomoMat T[5];
//function prototype
HomoMat JOINT_TO_MATRIX(int alpha, int a, int d, int theta);

// KIN: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Wrist Frame 
frameParam_t KIN(JOINT &conf) {
	// Assign configurations
	int theta_1 = conf[0];
	int theta_2 = conf[1];
	int d_3 = conf[2];
	int theta_4 = conf[3];

	// Define D-H frames
	int alpha[4] = { 0, 0, 0, 180 };
	int a[4] = { 0, L3, L4, 0 };
	int d[4] = { L1, L2, -(Lmax+d_3-L5), L5+L6 };
	int theta[4] = { theta_1, theta_2, 0, theta_4 };

	// Transformation matrices
	for (int i = 0; i < 4; i++) {
		T[i] = JOINT_TO_MATRIX(alpha[i], a[i], d[i], theta[i]);
	}

	HomoMat T_Wrist = T[0] * T[1] * T[2] * T[3];

	/* Debug code
	cout << "T[0]" << endl;
	T[0].PRINTT();
	cout << "T[1]" << endl;
	T[1].PRINTT();
	cout << "T[2]" << endl;
	T[2].PRINTT();
	cout << "T[3]" << endl;
	T[3].PRINTT();
	T_Wrist.PRINTT();
	*/

	frameParam_t temp = T_Wrist.ITOU();
	return temp;
}

// WHERE: Compute the forward kinematics of the planar 3R robot
// Input: Joint variables 
// Output: Tool Frame 
frameParam_t WHERE(JOINT &conf) {
	// Assign configurations
	int theta_1 = conf[0];
	int theta_2 = conf[1];
	int d_3 = conf[2];
	int theta_4 = conf[3];

	// Define D-H frames
	int alpha[5] = { 0, 0, 0, 180,0 };
	int a[5] = { 0, L3, L4, 0,0 };
	int d[5] = { L1, L2, -(Lmax+d_3-L5), L5+L6, (L7-L6+L8/2) };
	int theta[5] = { theta_1, theta_2, 0, theta_4, 0};

	// Transformation matrices
	for (int i = 0; i < 5; i++) {
		T[i] = JOINT_TO_MATRIX(alpha[i], a[i], d[i], theta[i]);
	}

	HomoMat T_Tool = T[0] * T[1] * T[2] * T[3]* T[4];
	frameParam_t temp = T_Tool.ITOU();
	return temp;

}


// Convert joint parameters into T matrix
HomoMat JOINT_TO_MATRIX(int alpha, int a, int d, int theta) {
	HomoMat temp;
	temp.homoMatrix[0][0] = cos(DEG2RAD(theta));
	temp.homoMatrix[0][1] = -sin(DEG2RAD(theta));
	temp.homoMatrix[0][2] =  0;
	temp.homoMatrix[0][3] =  a;

	temp.homoMatrix[1][0] = sin(DEG2RAD(theta))*cos(DEG2RAD(alpha));
	temp.homoMatrix[1][1] = cos(DEG2RAD(theta))*cos(DEG2RAD(alpha));
	temp.homoMatrix[1][2] = -sin(DEG2RAD(alpha));
	temp.homoMatrix[1][3] = -sin(DEG2RAD(alpha))*d;

	temp.homoMatrix[2][0] = sin(DEG2RAD(theta))*sin(DEG2RAD(alpha));
	temp.homoMatrix[2][1] = cos(DEG2RAD(theta))*sin(DEG2RAD(alpha));
	temp.homoMatrix[2][2] = cos(DEG2RAD(alpha));
	temp.homoMatrix[2][3] = cos(DEG2RAD(alpha))*d;

	temp.homoMatrix[3][0] = 0;
	temp.homoMatrix[3][1] = 0;
	temp.homoMatrix[3][2] = 0;
	temp.homoMatrix[3][3] = 1;
	return temp;
}