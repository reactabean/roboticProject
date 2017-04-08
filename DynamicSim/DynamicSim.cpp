// Library for Forward and Inverse Kinematics
// Created by Monica Li, April 2017


#include "DynamicSim.h"
using namespace std;

// todo list for funtion:
// add variable checks
// output values to file 
// move some assignments to the constant file 
// fix the dynamical matlab assignments

// Dynamic simulator for the manipulator, calculate the theta, theta_dot and theta_double_dot with given torque
//Euler-integration routine 
void update(JOINT &tau, JOINT &pos, JOINT &vel, JOINT &acc, double period)
{
	//todo
	//should do a torque check!

	// Local variables
	JOINT initialPosition, position, velocity, acceleration, dispPosition;
	HomoMat M, invM;
	JOINT V, G, F, temp;
	time_t before, after;
	double theta1, theta2, D3, theta4, thetadot1, thetadot2, Ddot3, thetadot4;
	bool work;

	// Assign initial conditions: pos = GetConfiguration(JOINT &conf), vel = [0,0,0,0]
	GetConfiguration(initialPosition);
	for (int i = 0; i < 4; i++)
	{
		if (i == 2) {
			position[i] = initialPosition[i];
		}
		else {
			position[i] = DEG2RAD(initialPosition[i]);
		}
		velocity[i] = 0;
	}

	before = clock();
	after = clock();

	while (period > difftime(after, before))
	{
		theta1 = position[0];
		theta2 = position[1];
		D3 = position[2];
		theta4 = position[3];

		thetadot1 = velocity[0];
		thetadot2 = velocity[1];
		Ddot3 = velocity[2];
		thetadot4 = velocity[3];


		// Matlab result==========================================================
		// M, V, G, F, invM
		M.homoMatrix[0][0] = M4*(l9*l9) + l3*(M2*l3*pow(cos(theta2), 2.0) - M2*l3*pow(sin(theta2), 2.0)) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
		M.homoMatrix[0][1] = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
		M.homoMatrix[0][2] = 0;
		M.homoMatrix[0][3] = -2 * l9 * l9 * M4;

		M.homoMatrix[1][0] = M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
		M.homoMatrix[1][1] = M3*(l4*l4) + M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4));
		M.homoMatrix[1][2] = 0;
		M.homoMatrix[1][3] = -2 * l9 * l9 * M4;

		M.homoMatrix[2][0] = 0;
		M.homoMatrix[2][1] = 0;
		M.homoMatrix[2][2] = M3 + M4;
		M.homoMatrix[2][3] = 0;

		M.homoMatrix[3][0] = -(M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)));
		M.homoMatrix[3][1] = -(M4*(l9*l9) + M4*l9*(l9 + l4*cos(theta4)));
		M.homoMatrix[3][2] = 0;
		M.homoMatrix[3][3] = 2 * l9 * l9 * M4;

		V[0] = -M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M2*(l3*l3)*(thetadot1*thetadot1)*cos(theta2)*sin(theta2)*2.0 + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
		V[1] = -M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
		V[2] = 0;
		V[3] = M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2));

		G[0] = 0;
		G[1] = 0;
		G[2] = -gravity*(M3 + M4);
		G[3] = 0;

		invM = M.TINVERT4();

		for (int i = 0; i < 4; i++)
		{
			F[i] = frictCoeff * velocity[i];
			// temp is (tau - V - G - F)
			temp[i] = tau[i] - V[i] - G[i] - F[i];
		}


		for (int i = 0; i < 4; i++)
		{
			//acceleration calculation
			acceleration[i] = 0;
			for (int j = 0; j < 4; j++)
			{
				//acceleration = invM * (tau - V - G - F); 
				//maybe we should zero out values if its small enough
				acceleration[i] = acceleration[i] + invM.homoMatrix[i][j] * temp[j];
			}

			if (abs(acceleration[i]) < 0.01) {
				acceleration[i] = 0;
			}

			//velocity and position calculation
			velocity[i] = velocity[i] + acceleration[i] * deltaT / 1000;
			position[i] = position[i] + velocity[i] * deltaT / 1000 + 0.5 * acceleration[i] * (deltaT / 1000) * (deltaT / 1000);

			// Send to output 
			pos[i] = position[i];
			vel[i] = velocity[i];
			acc[i] = acceleration[i];
		}

		//To format the radian represntation of postion into angles for display
		for (int i = 0; i < 4; i++)
		{
			if (i == 2) {
				pos[i] = position[i];
			}
			else {
				pos[i] = RAD2DEG(position[i]);
			}
		}

		// For debuging, delete on final delivery
		/*cout <<"=======dump of parameters========"<< endl;
		cout << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << endl;
		cout << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << endl;
		cout << acc[0] << "," << acc[1] << "," << acc[2] << "," << acc[3] << endl;*/
		work = DisplayConfiguration(pos);
		if (!work) cout << "Cannot Display Configuration " << endl;
		checkCubicValues(pos, vel, acc);

		Sleep(deltaT);
		after = clock();
	}

}


