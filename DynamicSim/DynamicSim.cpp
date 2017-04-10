// Library for Forward and Inverse Kinematics
// Created by Monica Li,edited by Andrew Nichol April 2017


#include "DynamicSim.h"
using namespace std;

// todo list for funtion:
//fix file outputs
//fix jointlimitmaxing


// pos,vel,acc are expected to be in degrees rather then radians
// Dynamic simulator for the manipulator, calculate the theta, theta_dot and theta_double_dot with given torque
//Euler-integration routine 
void update(JOINT &tau, JOINT &pos, JOINT &vel, JOINT &acc, double period, ofstream& myfile, time_t beforeC)
{

	// Local variables
	JOINT initialPosition, position, velocity, acceleration, dispPosition;
	HomoMat M, invM;
	JOINT V, G, F, temp;
	time_t before, after;
	double theta1, theta2, D3, theta4, thetadot1, thetadot2, Ddot3, thetadot4;
	bool work, error;

	// super timer variables
	LARGE_INTEGER frequency; 
    	LARGE_INTEGER t1,t2; 
    	double elapsedTime; 
    	QueryPerformanceFrequency(&frequency);

	//should do a torque check!
	torquecheck(tau);
	
	// Assign initial conditions: pos = GetConfiguration(JOINT &conf), vel = whatever gets passed in //this is because we cannot make anyassumptions about the current speed of the robot when torque is applied
	GetConfiguration(initialPosition);
	for (int i = 0; i < 4; i++)
	{
		if (i == 2) {
			position[i] = initialPosition[i];
			velocity[i] = vel[i];
		}
		else {
			position[i] = DEG2RAD(initialPosition[i]);
			velocity[i] = DEG2RAD(vel[i]);
		}
	}

	before = clock();
	after = clock();
	while (period > difftime(after, before))
	{

		// Matlab dyn equation result==========================================================
		// M, V, G, F, invM
		M = Mfun(position, velocity);
		Vfun(V,position, velocity);
		Gfun(G);
		Ffun(F, velocity);
		//

		invM = M.TINVERT4();

		//M.PRINTT();
		//invM.PRINTT();

		for (int i = 0; i < 4; i++)
		{
			// temp is (tau - V - G - F)
			temp[i] = tau[i] - V[i] - G[i] - F[i];
		}

		//cout << "temp" << i << "," << temp[i] << endl;

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

			// Send to output and format it so it is in degrees
			if (i == 2) {
				pos[i] = position[i];
				vel[i] = velocity[i];
				acc[i] = acceleration[i];
			}
			else {
				pos[i] = RAD2DEG(position[i]);
				vel[i] = RAD2DEG(velocity[i]);
				acc[i] = RAD2DEG(acceleration[i]);
			}
		}

		// For debuging, delete on final delivery
		/*cout <<"=======dump of parameters========"<< endl;
		cout << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << endl;
		cout << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << endl;
		cout << acc[0] << "," << acc[1] << "," << acc[2] << "," << acc[3] << endl;*/
		after = clock();
		printPosVelAccToFile(myfile, pos, vel, acc, difftime(after, beforeC), tau);
		
		//Not sure if we need this
		/*error = !checkCubicValues(pos, vel, acc);

		//for debugging 
		//do not know if this is a good idea or not
		if (error == 1) {
			jointposMaxing(pos,position);
			cout << "In violation of joint Configuration, not letting it move further" << endl;
		}*/

		work = DisplayConfiguration(pos);
		
		if (!work){
			cout << "Cannot Display Configuration " << endl;
		}

		QueryPerformanceCounter(&t1);
		QueryPerformanceCounter(&t2);
		elapsedTime=(float)(t2.QuadPart-t1.QuadPart)/frequency.QuadPart;
		while(elapsedTime*1000<deltaT){
			QueryPerformanceCounter(&t2);
			elapsedTime=(float)(t2.QuadPart-t1.QuadPart)/frequency.QuadPart; 
		}
		after = clock();
	}

}


void printPosVelAccToFile(ofstream &outputFile, JOINT &pos, JOINT &vel, JOINT &acc, double time, JOINT &tau) {
	outputFile << (isnan(time) ? 0 : time) << "," << (isnan(pos[0]) ? 0 : pos[0]) << "," << (isnan(pos[1]) ? 0 : pos[1]) << "," << (isnan(pos[2]) ? 0 : pos[2]) << "," << (isnan(pos[3]) ? 0 : pos[3]) << "," << (isnan(vel[0]) ? 0 : vel[0]) << "," << (isnan(vel[1]) ? 0 : vel[1]) << "," << (isnan(vel[2]) ? 0 : vel[2]) << "," << (isnan(vel[3]) ? 0 : vel[3]) << "," << (isnan(acc[0]) ? 0 : acc[0]) << "," << (isnan(acc[1]) ? 0 : acc[1]) << "," << (isnan(acc[2]) ? 0 : acc[2]) << "," << (isnan(acc[3]) ? 0 : acc[3]) << "," << (isnan(tau[0]) ? 0 : tau[0]) << "," << (isnan(tau[1]) ? 0 : tau[1]) << "," << (isnan(tau[2]) ? 0 : tau[2]) << "," << (isnan(tau[3]) ? 0 : tau[3]) << endl;
	//outputFile << time << "," << (isnan(pos[0]) ? 0 : pos[0]) << "," << (isnan(pos[1]) ? 0 : pos[1]) << "," << (isnan(pos[2]) ? 0 : pos[2]) << "," << (isnan(pos[3]) ? 0 : pos[3]) << "," << (isnan(vel[0]) ? 0 : vel[0]) << "," << (isnan(vel[1]) ? 0 : vel[1]) << "," << (isnan(vel[2]) ? 0 : vel[2]) << "," << (isnan(vel[3]) ? 0 : vel[3]) << "," << (isnan(acc[0]) ? 0 : acc[0]) << "," << (isnan(acc[1]) ? 0 : acc[1]) << "," << (isnan(acc[2]) ? 0 : acc[2]) << "," << (isnan(acc[3]) ? 0 : acc[3]) << "," << (isnan(tau[0]) ? 0 : tau[0]) << "," << (isnan(tau[1]) ? 0 : tau[1]) << "," << (isnan(tau[2]) ? 0 : tau[2]) << "," << (isnan(tau[3]) ? 0 : tau[3]) << endl;
	// matlab doesnt like nan values
}

void Gfun( JOINT &G){
	G[0] = 0;
	G[1] = 0;
	G[2] = -gravity*(M3 + M4);
	G[3] = 0;
}

void Vfun(JOINT &V, JOINT &position, JOINT &velocity){
		double theta1, theta2, D3, theta4, thetadot1, thetadot2, Ddot3, thetadot4;

		theta1 = position[0];
		theta2 = position[1];
		D3 = position[2];
		theta4 = position[3];

		thetadot1 = velocity[0];
		thetadot2 = velocity[1];
		Ddot3 = velocity[2];
		thetadot4 = velocity[3];

		V[0] = -M4*l9*(sin(theta4)*(l4*(thetadot1*thetadot1) + l4*(thetadot2*thetadot2) + l3*(thetadot1*thetadot1)*cos(theta2) + l4*thetadot1*thetadot2*2.0) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
		V[1] = -M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2)) + M3*l3*l4*(thetadot1*thetadot1)*sin(theta2);
		V[2] = 0;
		V[3] = M4*l9*(sin(theta4)*(l4*pow(thetadot1 + thetadot2, 2.0) + l3*(thetadot1*thetadot1)*cos(theta2)) - l3*(thetadot1*thetadot1)*cos(theta4)*sin(theta2));
}

HomoMat Mfun(JOINT &position, JOINT &velocity){
		double theta1, theta2, D3, theta4, thetadot1, thetadot2, Ddot3, thetadot4;
		HomoMat M;
		theta1 = position[0];
		theta2 = position[1];
		D3 = position[2];
		theta4 = position[3];

		thetadot1 = velocity[0];
		thetadot2 = velocity[1];
		Ddot3 = velocity[2];
		thetadot4 = velocity[3];	

		M.homoMatrix[0][0] = M2*(l3*l3) + M4*(l9*l9) + M4*l9*(l9 + cos(theta4)*(l4 + l3*cos(theta2)) + l3*sin(theta2)*sin(theta4)) + M3*l4*(l4 + l3*cos(theta2));
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

		return M;
}

void Ffun (JOINT &F, JOINT &velocity) {

		for (int i = 0; i < 4; i++)
		{
			F[i] = frictCoeff * velocity[i];
		}
}

//used to keep robot moving past the joint limits. 
void jointposMaxing( JOINT&pos, JOINT&position){


	if (pos[0] > HIGHTHEATA1)	{pos[0]= HIGHTHEATA1;}
	if (pos[0] < LOWTHEATA1)	{pos[0]= LOWTHEATA1;}
	
	if (pos[1] > HIGHTHEATA2)	{pos[1]= HIGHTHEATA2;}
	if (pos[1] < LOWTHEATA12)	{pos[1]= LOWTHEATA12;}
	
	if (pos[2] > HIGHDISTANCE3)	{pos[2]= HIGHDISTANCE3;}
	if (pos[2] < LOWDISTANCE3)	{pos[2]= LOWDISTANCE3;}

	if (pos[3] > HIGHTHEATA4)	{pos[3]= HIGHTHEATA4;}
	if (pos[3] < LOWTHEATA4)	{pos[3]= LOWTHEATA4;}

	for (int i = 0; i < 4; i++)
	{
		if (i == 2) {
			position[i] = pos[i];
		}
		else {
			position[i] = DEG2RAD(pos[i]);
		}
	}


}

void torquecheck( JOINT& tau){
	bool error; 
	error = 0;

	if (abs(tau[0])>ROTARYTORQUE | abs(tau[1])>ROTARYTORQUE | abs(tau[2])>LINEARTORQUE | abs(tau[3])>ROTARYTORQUE)
	{
		cout <<  " Note Torque being applied is past the intended limits at: " << tau[0] << "," << tau[1] << "," << tau[2] << "," << tau[3] <<  endl;
		error = 1;
	}

}