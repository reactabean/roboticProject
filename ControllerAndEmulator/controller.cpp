// Library for Controller
// Created by Andrew Nichol, april 2017

#include "controller.h"
using namespace std;


//todo 
// write f,v,g,F funciton, and also properly have initial values 
// need to check rad/deg units

bool moveCont(JOINT &dConf, JOINT &dVel, JOINT &dAcc,JOINT &tVel){
	//tVel are passed in to act like initial values
	//local variables
	bool error;
	JOINT tConf, tAcc,temp,temp2;
	time_t before, after;
	int kV[4] = { KV1, KV2, KV3, KV4 };
	int kP[4] = { KP1, KP2, KP3, KP4 };
	JOINT torque = { 0, 0, 0, 0 };
	HomoMat M;
	JOINT V, G, F;
	
	//initialize error state as false
	error = false;

	//get current position before loop 
	error = GetConfiguration(tConf);

	//get current program time count
	before = clock();
	after = clock();

	//execute for 20 msecs
	while (CONTROLLERTIME > difftime(after, before)){
		
		//todo write these functions
		M = Mfun(tConf, tVel);
		Vfun(V,tConf, tVel);
		Gfun(G);
		Ffun(F, tVel);

		for (int i = 0; i < 4; i++) {
			//non partitioned stage
			temp[i] = (kP[i]*(dConf[i] - tConf[i]) + kV[i]*(dVel[i] - tVel[i]) + dAcc[i]);
			
			//matrix multiplicaiton
			temp2[i] = 0;
			for (int j = 0; j < 4; j++)
			{
				temp2[i] = temp2[i] + M.homoMatrix[i][j] * temp[j];
			}

			//partitioned stage
			torque[i] = temp2[i] + V[i] +G[i] + F[i];   
		}
		
		//todo: should have error check on torque values

		//applies a new torque to the emulator 
		//this executes for 2 ms
		update(torque, tConf, tVel, tAcc, CONTROLLERSAMPTIME);
		
		//update timer
		after = clock();
	}

	return error;
}