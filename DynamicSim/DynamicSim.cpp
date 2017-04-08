// Library for Forward and Inverse Kinematics
// Created by Monica Li, April 2017


#include "DynamicSim.h"
using namespace std;

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
	double deltaT = 10;
	double v;
	double theta1, theta2, D3, theta4, thetadot1, thetadot2, Ddot3, thetadot4;
	bool work;

	// Friction coefficient
	v = 0.5;

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


		// To do: Assign the matrices for dynamic equation according to matlab result
		// M,V,G,F
		M.homoMatrix[0][0] = M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2)) + M2*(l3*l3)*cos(theta2*2.0);
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

		V[0] = (l3*l3)*M2*(thetadot1*thetadot1)*sin(theta2*2.0) + l3*l4*M3*(thetadot1*thetadot1)*sin(theta2) - l4*l9*M4*(thetadot1*thetadot1)*sin(theta4) - l4*l9*M4*(thetadot2*thetadot2)*sin(theta4) + l3*l9*M4*(thetadot1*thetadot1)*sin(theta2 - theta4) - l4*l9*M4*thetadot1*thetadot2*sin(theta4)*2.0;
		V[1] = l3*l4*M3*(thetadot1*thetadot1)*sin(theta2) - l4*l9*M4*(thetadot1*thetadot1)*sin(theta4) - l4*l9*M4*(thetadot2*thetadot2)*sin(theta4) + l3*l9*M4*(thetadot1*thetadot1)*sin(theta2 - theta4) - l4*l9*M4*thetadot1*thetadot2*sin(theta4)*2.0;
		V[2] = 0;
		V[3] = l9*M4*(l4*(thetadot1*thetadot1)*sin(theta4) + l4*(thetadot2*thetadot2)*sin(theta4) - l3*(thetadot1*thetadot1)*sin(theta2 - theta4) + l4*thetadot1*thetadot2*sin(theta4)*2.0);

		G[0] = M3*gravity*l4*cos(theta1 + theta2) + M2*gravity*l3*cos(theta1 + theta2*2.0) + M4*gravity*l9*cos(theta1 + theta2 - theta4);
		G[1] = M3*gravity*l4*cos(theta1 + theta2) + M4*gravity*l9*cos(theta1 + theta2 - theta4);
		G[2] = 0;
		G[3] = -M4*gravity*l9*cos(theta1 + theta2 - theta4);

		// Inverse the mass matrix 
		invM.homoMatrix[0][0] = 1.0 / (l3*l3) / (M2*cos(theta2*2.0));
		invM.homoMatrix[0][1] = -1.0 / (l3*l3) / (M2*cos(theta2*2.0));
		invM.homoMatrix[0][2] = 0;
		invM.homoMatrix[0][3] = 0;

		invM.homoMatrix[1][0] = -(1.0 / (l3*l3)*(l4 + l3*cos(theta2))) / (M2*l4*(pow(cos(theta2), 2.0)*2.0 - 1.0));
		invM.homoMatrix[1][1] = (1.0 / (l3*l3)*1.0 / (l4*l4)*(M3*(l4*l4) + M2*(l3*l3)*(pow(cos(theta2), 2.0)*2.0 - 1.0) + M3*l3*l4*cos(theta2))) / (M2*M3*(pow(cos(theta2), 2.0)*2.0 - 1.0));
		invM.homoMatrix[1][2] = 0;
		invM.homoMatrix[1][3] = 1.0 / (l4*l4) / M3;

		invM.homoMatrix[2][0] = 0;
		invM.homoMatrix[2][1] = 0;
		invM.homoMatrix[2][2] = 1.0 / (M3 + M4);
		invM.homoMatrix[2][3] = 0;

		invM.homoMatrix[3][0] = (-l9*cos(theta2) + l4*sin(theta2)*sin(theta4)*(1.0 / 2.0)) / (M2*l3*l4*l9*(pow(cos(theta2), 2.0)*2.0 - 1.0));
		invM.homoMatrix[3][1] = (1.0 / (l4*l4)*(-M3*(l4*l4)*sin(theta2)*sin(theta4) + M3*l4*l9*cos(theta2)*2.0 + M2*l3*l9*(pow(cos(theta2), 2.0)*2.0 - 1.0)*2.0 + M2*l3*l4*cos(theta4)*(pow(cos(theta2), 2.0)*2.0 - 1.0))*(1.0 / 2.0)) / (M2*M3*l3*l9*(pow(cos(theta2), 2.0)*2.0 - 1.0));
		invM.homoMatrix[3][2] = 0;
		invM.homoMatrix[3][3] = (1.0 / (l4*l4)*1.0 / (l9*l9)*(M3*(l4*l4) + M4*(l9*l9)*2.0 + M4*l4*l9*cos(theta4))*(1.0 / 2.0)) / (M3*M4);

		for (int i = 0; i < 4; i++)
		{
			F[i] = v * velocity[i];
			// temp is (tau - V - G - F)
			temp[i] = tau[i] - V[i] - G[i] - F[i];
		}


		for (int i = 0; i < 4; i++)
		{
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
		cout << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << endl;
		work = DisplayConfiguration(pos);
		if (!work) cout << " Doesn't work for DisplayConfiguration " << endl;

		Sleep(deltaT);
		after = clock();
	}

}


